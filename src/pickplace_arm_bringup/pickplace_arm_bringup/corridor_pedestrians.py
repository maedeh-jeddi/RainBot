#!/usr/bin/env python3
"""Walk hospital pedestrians up and down their corridor lanes.

Each pedestrian is a models/pedestrian instance carrying gz-sim's
VelocityControl system, so driving one is just publishing a Twist on its
/model/<name>/cmd_vel (bridged in gazebo.launch.py). This node patrols them
along a lane and turns them round at each end.

VELOCITY, NOT FORCE, and NOT teleporting:
  * A force-driven 80 kg upright mesh topples or spins the first time anything
    touches it.
  * Teleporting the pose each tick makes the person pass through the robot
    instead of colliding with it, which would quietly turn "the robot avoided
    the pedestrian" into a claim nothing could test.
  VelocityControl leaves the collision real while keeping the gait steady.

A pedestrian never turns: it walks backwards along its lane instead of
about-facing, so its yaw is fixed at spawn. That keeps the frame maths trivial -
VelocityControl applies the twist in the MODEL frame, and with a constant yaw
the forward and lateral world directions are constants too.

The lane is held closed-loop from /model/<name>/pose. A pedestrian shoved
sideways in a collision would otherwise walk the rest of the run down the middle
of the corridor, and "it passed a pedestrian standing in its lane" would stop
describing what the run actually did.

LANES ARE PER WORLD, selected by the `world` parameter, because the two
hospitals have their corridors on different axes: hospital_lab's runs along x,
the AWS building's two side corridors run along y. Both were chosen against
measured wall clearance, and in both the two pedestrians patrol NON-OVERLAPPING
stretches so the robot meets one at a time and always has most of the corridor
to pass on rather than having to thread between two people at once.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist

# name, yaw (rad, fixed), lane point (a world x,y ON the lane centre line),
# patrol extent measured along the heading from that point (lo, hi), speed m/s.
LANES = {
    # 4.0 m corridor along +x, walls at y = +/-2.0.
    'hospital_lab': [
        ('pedestrian_0', 0.0, (2.5, 1.10), -1.5, 1.5, 0.85),
        ('pedestrian_1', 0.0, (7.0, -1.10), -1.5, 1.5, 0.85),
    ],
    # AWS hospital: the east side corridor runs along y from about y=-30 to
    # y=-3 with ~1.4 m of clearance either side of x=9.5, i.e. ~2.8 m wide.
    # The two lanes sit either side of its centre line and the patrol stretches
    # do not overlap, so only one pedestrian is ever alongside the robot.
    # Both lanes sit ON the corridor's measured centre line, x = 9.5, where
    # clearance was sampled at 1.4 m or better. An earlier version offset them
    # to x = 8.9 and x = 10.1 to give the robot the middle, and put one at
    # y = -10.5 - and both promptly jammed after ~0.2 m, because neither offset
    # was re-measured and y ~ -10 is a pinch point where clearance collapses to
    # 0.10 m. The clear stretches along x = 9.5 are y -30..-26, -22..-12 and
    # -8..-3; each patrol below stays inside one of them, and they do not
    # overlap, so the robot still meets one pedestrian at a time.
    #
    # The -8..-3 band came out of a 1 m-spaced sweep and does NOT survive
    # contact with the simulator: a pedestrian put there drifted 0.14 m off the
    # centre line and jammed. Both now use stretches that were measured open at
    # 2.9 m and 3.4 m of clearance and are confirmed walkable.
    'aws_hospital': [
        ('pedestrian_0', math.pi / 2, (9.5, -17.0), -4.0, 4.0, 0.85),
        ('pedestrian_1', math.pi / 2, (9.5, -28.0), -2.0, 2.0, 0.85),
    ],
}

LANE_GAIN = 1.2              # 1/s, how hard to steer back onto the lane
MAX_LANE_CORRECTION = 0.35   # m/s of sideways correction


class CorridorPedestrians(Node):
    def __init__(self):
        super().__init__('corridor_pedestrians')
        self.declare_parameter('world', 'hospital_lab')
        world = self.get_parameter('world').value
        if world not in LANES:
            self.get_logger().error(
                f'no pedestrian lanes defined for world {world!r}; '
                f'known: {sorted(LANES)}')
            world = 'hospital_lab'

        self.state = {}
        self.pubs = {}
        for name, yaw, lane_pt, s_lo, s_hi, speed in LANES[world]:
            self.pubs[name] = self.create_publisher(
                Twist, f'/model/{name}/cmd_vel', 10)
            self.state[name] = {
                'fwd': (math.cos(yaw), math.sin(yaw)),
                'lat': (-math.sin(yaw), math.cos(yaw)),
                'origin': lane_pt,
                's_lo': s_lo, 's_hi': s_hi,
                'speed': speed, 'dir': 1,
                'pos': lane_pt,
            }
            self.create_subscription(
                Pose, f'/model/{name}/pose',
                lambda msg, n=name: self._on_pose(n, msg), 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'[{world}] walking {len(self.state)} pedestrians: '
            + ', '.join(f'{n} about ({p[0]:.1f},{p[1]:.1f}) '
                        f'+/-{hi:.1f} m @ {sp} m/s'
                        for n, _, p, _, hi, sp in LANES[world]))

    def _on_pose(self, name, msg):
        self.state[name]['pos'] = (msg.position.x, msg.position.y)

    def _tick(self):
        for name, st in self.state.items():
            ox, oy = st['origin']
            px, py = st['pos']
            dx, dy = px - ox, py - oy
            s = dx * st['fwd'][0] + dy * st['fwd'][1]      # along the lane
            lat = dx * st['lat'][0] + dy * st['lat'][1]    # off the lane

            if st['dir'] > 0 and s >= st['s_hi']:
                st['dir'] = -1
            elif st['dir'] < 0 and s <= st['s_lo']:
                st['dir'] = 1

            cmd = Twist()
            cmd.linear.x = st['speed'] * st['dir']
            cmd.linear.y = max(-MAX_LANE_CORRECTION,
                               min(MAX_LANE_CORRECTION, -LANE_GAIN * lat))
            self.pubs[name].publish(cmd)

    def stop(self):
        for pub in self.pubs.values():
            pub.publish(Twist())


def main():
    rclpy.init()
    node = CorridorPedestrians()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
