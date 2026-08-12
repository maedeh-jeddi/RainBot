#!/usr/bin/env python3
"""Drive a fixed tour of hospital_lab.sdf so slam_toolbox can map it.

This replaces a human on teleop_key for the mapping run. Every waypoint below
was checked against the measured footprint of every prop in the world, so the
robot keeps at least ~0.6 m of clearance everywhere it has to turn (the Husky's
circumscribed radius is sqrt(0.495^2 + 0.335^2) = 0.598 m) and ~0.5 m where it
only drives straight past something.

Poses come from the map -> base_link transform, i.e. the SLAM-corrected pose,
not raw odometry: over a ~70 m tour the skid-steer's odom drift would otherwise
walk the waypoints away from the real building and into the furniture.

slam_toolbox puts the map origin at the robot's STARTING pose, so map
coordinates are world coordinates shifted by -SPAWN. The tour is written in
world coordinates and converted once, in world_to_map().

A forward LIDAR watchdog abandons a waypoint if driving on would hit something;
a mapping run that skips a waypoint is recoverable, a run that shoves a trolley
across the room and maps it in two places is not.

Run it against a live mapping session:

    WORLD=hospital_lab.sdf SPAWN_X=-6.5 SPAWN_Y=0.0 \
        ros2 launch pickplace_arm_bringup mapping.launch.py
    ros2 run pickplace_arm_bringup hospital_map_drive     # 2nd terminal
    ros2 run nav2_map_server map_saver_cli \
        -f src/pickplace_arm_bringup/maps/hospital_lab

Before changing TOUR, re-check it:

    ros2 run pickplace_arm_bringup hospital_route_check

The tour that produced the shipped map ran all 33 waypoints with no watchdog
trip and no timeout; AMCL on the resulting map sits ~0.03 m off ground truth.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

SPAWN_X, SPAWN_Y = -6.5, 0.0

# Tour in WORLD coordinates. Lab sweep, then the corridor (snaking around the
# two pedestrians and the trolleys), then a ward loop, then the whole corridor
# back again -- the return leg is what gives slam_toolbox its loop closure.
TOUR = [
    # --- lab ---
    (-6.5, -1.8), (-4.0, -1.5), (-2.5, -0.5), (-4.8, 1.0), (-6.5, 1.0),
    (-5.0, 2.0), (-5.0, 0.5), (-3.0, 0.0), (-1.8, 0.0),
    # --- corridor, west to east; the lane snakes to clear the two
    #     pedestrians and the trolleys, all of which stand against a wall ---
    (-0.5, 0.0), (0.5, 0.2), (1.8, -0.2), (3.3, -0.6), (5.0, 0.1),
    (6.8, 0.7), (8.3, 0.3), (9.8, 0.0),
    # --- ward. NOT a perimeter loop: the delivery bench sits in the middle
    #     of the room and the gaps either side of it are too narrow for the
    #     Husky (0.88 m between the bench and the bedside table to the south;
    #     ward_scrubs blocks the northern lane outright). These are viewpoints
    #     instead, which is enough - the LIDAR reaches 8 m and the room is
    #     7.9 m across, so every wall is in range from here. The pocket
    #     directly behind the bench stays unknown, and that is fine: the
    #     mission never drives there. ---
    (10.8, 0.0), (11.8, 0.0), (12.5, -1.0), (13.0, 0.0), (11.8, 0.0),
    # --- corridor, east to west ---
    (9.8, 0.0), (8.3, 0.3), (6.8, 0.7), (5.0, 0.1), (3.3, -0.6),
    (1.8, -0.2), (0.5, 0.2), (-0.5, 0.0),
    # --- back into the lab, closing the loop on the start pose ---
    (-2.5, 0.0), (-5.0, 0.0), (-6.5, 0.0),
]

REACH_TOL = 0.25        # m, waypoint counts as reached
TURN_IN_PLACE = 0.6     # rad, above this heading error stop and pivot
MAX_LIN = 0.30          # m/s, slow enough for the scan matcher
MAX_ANG = 0.5           # rad/s
# Watchdog geometry, in the LIDAR frame. The first version of this compared
# raw range inside a +/- 50 deg wedge, which asks "is anything near me?" - the
# wrong question. It tripped at 0.54 m while the robot was legitimately driving
# PAST the corridor's parking trolley with 0.52 m of lateral clearance, because
# something you pass beside is still inside a 50 deg cone.
#
# The question that matters is "will I hit it if I keep going straight", so the
# test is now whether a return lands inside a rectangle projected ahead of the
# robot: half the Husky's width plus margin, extending far enough forward to
# stop in. Pivots are not covered by this and do not need to be - every
# waypoint was verified to clear the robot's 0.598 m circumscribed radius.
SAFE_HALF_WIDTH = 0.42  # m, Husky half width 0.335 + margin
SAFE_AHEAD = 0.45       # m, projected ahead of the LIDAR
WAYPOINT_TIMEOUT = 90.0  # s
MAX_TRIPS = 3           # watchdog trips tolerated before giving up entirely


def world_to_map(x, y):
    return x - SPAWN_X, y - SPAWN_Y


def ang_norm(a):
    return math.atan2(math.sin(a), math.cos(a))


class MapDrive(Node):
    def __init__(self):
        super().__init__('map_drive')
        self.pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.blocked_at = None
        self.trips = 0

    def on_scan(self, msg):
        """Nearest return lying inside the forward safety rectangle, if any."""
        nearest = None
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            x, y = r * math.cos(a), r * math.sin(a)
            if 0.0 < x <= SAFE_AHEAD and abs(y) <= SAFE_HALF_WIDTH:
                if nearest is None or x < nearest:
                    nearest = x
        self.blocked_at = nearest

    def pose(self):
        """(x, y, yaw) of base_link in the map frame, or None if TF is not up."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def stop(self):
        self.pub.publish(Twist())

    def drive(self):
        # Wait for SLAM to publish map -> odom before trusting any pose.
        self.get_logger().info('waiting for map -> base_link ...')
        while rclpy.ok() and self.pose() is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info('TF up, starting tour')

        for idx, (wx, wy) in enumerate(TOUR):
            gx, gy = world_to_map(wx, wy)
            start = self.get_clock().now()
            self.get_logger().info(
                f'[{idx + 1}/{len(TOUR)}] -> world ({wx:.1f}, {wy:.1f})')

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                p = self.pose()
                if p is None:
                    continue
                x, y, yaw = p

                dx, dy = gx - x, gy - y
                dist = math.hypot(dx, dy)
                if dist < REACH_TOL:
                    break

                elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
                if elapsed > WAYPOINT_TIMEOUT:
                    self.get_logger().warn(
                        f'waypoint {idx + 1} timed out at {dist:.2f} m, '
                        f'moving on')
                    break

                herr = ang_norm(math.atan2(dy, dx) - yaw)
                cmd = Twist()
                if abs(herr) > TURN_IN_PLACE:
                    # Pivot on the spot. The watchdog is deliberately NOT
                    # consulted here: a robot that has arrived somewhere and
                    # not yet turned is usually still pointing at whatever it
                    # drove up to, so testing the forward rectangle mid-pivot
                    # reports a collision it is in the middle of turning away
                    # from. Pivot safety comes from the route check instead -
                    # every waypoint clears the 0.598 m circumscribed radius.
                    cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, 1.0 * herr))
                else:
                    if self.blocked_at is not None:
                        # About to drive forward into something. Stop, give up
                        # on this waypoint and try the next one rather than
                        # ending the run: one marginal approach angle should
                        # not throw away a six-minute mapping pass.
                        self.stop()
                        self.trips += 1
                        self.get_logger().warn(
                            f'watchdog: obstacle {self.blocked_at:.2f} m dead '
                            f'ahead approaching world ({wx:.1f}, {wy:.1f}), '
                            f'skipping (trip {self.trips}/{MAX_TRIPS})')
                        if self.trips >= MAX_TRIPS:
                            self.get_logger().error(
                                'ABORT: too many watchdog trips')
                            return False
                        break
                    cmd.linear.x = max(0.05, min(MAX_LIN, 0.6 * dist))
                    cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, 1.2 * herr))
                self.pub.publish(cmd)

            self.stop()

        self.stop()
        self.get_logger().info('tour complete')
        return True


def main():
    rclpy.init()
    node = MapDrive()
    ok = False
    try:
        ok = node.drive()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
