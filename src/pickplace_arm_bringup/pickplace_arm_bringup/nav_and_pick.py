#!/usr/bin/env python3
"""Autonomous navigate-and-pick.

Builds on the mobile search-and-pick, but replaces the naive "spin then drive
straight at the box" long-range motion with real Nav2 goal-based navigation
(global path planning + costmap obstacle avoidance). Once Nav2 has driven the
base to a pose ~APPROACH_DIST in front of the box, control is handed off to
the already-verified visual servo (SearchAndPick.search_and_approach) for the
final precise positioning and then the inherited arm run() pick sequence.

Flow:
  1) Arm -> search pose; spin in place (high-priority /cmd_vel_search via
     twist_mux) until the box is first detected.
  2) Transform the box from base_link into the map frame (slam_toolbox
     supplies map->odom; diff_drive supplies odom->base_link).
  3) Compute an approach goal ~APPROACH_DIST from the box, on the robot->box
     ray, facing the box; send it as a Nav2 NavigateToPose goal. Nav2 plans a
     path and follows it, steering around obstacles (base-level avoidance).
  4) On arrival, run the inherited visual search_and_approach() to servo the
     last short distance precisely, then run() to scan/grasp/carry/place.

The arm keeps its existing MoveIt collision-aware planning unchanged.
"""
import math
import time
import threading

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from action_msgs.msg import GoalStatus
import tf2_ros
import tf2_geometry_msgs  # registers do_transform_point for PointStamped

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap

from pickplace_arm_bringup.pick_and_place import scan_quat
from pickplace_arm_bringup.search_and_pick import (
    SearchAndPick, SEARCH_POSITION, SEARCH_PITCH, SPIN_ANGULAR, SPIN_STEP_RAD,
    SPIN_SETTLE_SEC, SPIN_STEPS_PER_REV)

# How far in front of the box Nav2 should stop. ~1.4 m keeps the box clear of
# the front camera's near blind zone (it is pitched down, so a box ~1 m away
# falls below its view) -- if Nav2 stopped closer the front-cam approach saw
# nothing and spun the chassis to hunt for the box. At ~1.4 m the box sits
# comfortably in the front camera's [~1.0, 2.5] m range, so the visual servo
# drives straight in without rotating. Also far enough that Nav2 settles on an
# open spot (no recovery Spin). The servo closes the last ~1 m precisely.
APPROACH_DIST = 1.4
NAV_TIMEOUT_SEC = 120.0

# Coverage search: if the box isn't seen from the start, drive (via Nav2, so
# walls/obstacles are avoided) to a ring of scan waypoints in the map frame
# (anchored at the robot's start) and spin-scan at each. Spaced so the camera's
# ~1.1 m detection reach sweeps the whole room; kept within +/-1.8 m so the
# robot (radius 0.25 + 0.30 inflation) stays clear of walls at +/-3 m. This
# replaces blind dead-reckoned creeping, which was unreliable once the
# skid-steer's heading drifted -- now that odom/heading is IMU-corrected and
# Nav2 localizes well, driving to explicit map waypoints is dependable.
EXPLORE_WAYPOINTS = [
    (1.8, 0.0), (1.3, 1.3), (0.0, 1.8), (-1.3, 1.3),
    (-1.8, 0.0), (-1.3, -1.3), (0.0, -1.8), (1.3, -1.3),
]


class NavAndPick(SearchAndPick):
    def __init__(self):
        super().__init__()
        # The inherited cmd_vel publisher already targets the diff drive
        # controller directly. Navigation (Nav2) and this node's spin/visual
        # servo run in separate, non-overlapping phases, so they can share that
        # topic without a twist_mux arbitrating between them.
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Costmap clearing. Relative names so they land under the robot's
        # namespace exactly like every other client here.
        self._clear_local = self.create_client(
            ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')
        self._clear_global = self.create_client(
            ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')
        self.get_logger().info('Nav-and-pick node ready')

    # --- scan one full revolution in place, return the box if seen ----------
    def scan_in_place(self):
        log = self.get_logger()
        for _ in range(SPIN_STEPS_PER_REV):
            det = self.detect_box_pose(timeout_sec=1.0)
            if det is not None:
                bx, by, _ = det
                log.info(f'[explore] box seen: dist={math.hypot(bx,by):.2f}m '
                          f'bearing={math.degrees(math.atan2(by,bx)):.1f}deg')
                return det
            self._rotate_step(SPIN_STEP_RAD)
        return self.detect_box_pose(timeout_sec=1.0)

    # --- coverage search: scan at start, then at Nav2-reached waypoints ------
    def explore_and_find(self):
        log = self.get_logger()
        sx, sy, sz = SEARCH_POSITION
        self.move_pose(sx, sy, sz, label='search-scan',
                       quat_xyzw=scan_quat(SEARCH_PITCH))

        det = self.scan_in_place()
        if det is not None:
            return det

        for i, (wx, wy) in enumerate(EXPLORE_WAYPOINTS):
            log.info(f'[explore] -> waypoint {i + 1}/{len(EXPLORE_WAYPOINTS)} '
                      f'map({wx:.1f},{wy:.1f})')
            goal = self.make_map_goal(wx, wy, math.atan2(wy, wx))
            if not self.navigate_to(goal, timeout_sec=60.0):
                log.warn(f'[explore] could not reach waypoint {i + 1} -- skipping')
                continue
            det = self.scan_in_place()
            if det is not None:
                return det

        log.error('[explore] box not found at any waypoint')
        return None

    # --- transform a base_link point into the map frame ---------------------
    def box_in_map(self, bx, by):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.tf_frame('base_link'), rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'[nav] map<-base_link TF failed: {e}')
            return None
        pt = PointStamped()
        pt.header.frame_id = 'base_link'
        pt.point.x, pt.point.y, pt.point.z = bx, by, 0.0
        pm = tf2_geometry_msgs.do_transform_point(pt, tf)
        return pm.point.x, pm.point.y

    # --- and the other way: a map point in the robot's own frame -------------
    def map_point_in_base(self, mx, my):
        """Where a KNOWN map point sits relative to the robot right now.

        box_in_map above answers "the camera sees something there, where is that
        in the building?". This answers the opposite question, which is the one
        a handover asks: the transfer point is a fixed map coordinate agreed
        between two robots, and the arm has to be told where it is in base_link.

        NOT A CONSTANT, AND THAT IS THE POINT. Nav2 parks with a real tolerance
        (nav2_params.yaml: xy 0.20), so a robot standing "at" the handover pose
        may be up to 0.20 m off it in any direction. Commanding the arm to the
        nominal offset instead of the measured one puts the payload down that
        far from where the other robot expects to find it - and the whole margin
        between the transfer point and MAX_REACH_X is 0.075 m.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_frame('base_link'), 'map', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'[nav] base_link<-map TF failed: {e}')
            return None
        pt = PointStamped()
        pt.header.frame_id = 'map'
        pt.point.x, pt.point.y, pt.point.z = mx, my, 0.0
        pb = tf2_geometry_msgs.do_transform_point(pt, tf)
        return pb.point.x, pb.point.y

    def robot_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.tf_frame('base_link'), rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'[nav] robot pose TF failed: {e}')
            return None
        t = tf.transform.translation
        return t.x, t.y

    def make_map_goal(self, mx, my, yaw):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = mx
        goal.pose.position.y = my
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def compute_approach_goal(self, box_map, robot_map):
        bx, by = box_map
        rx, ry = robot_map
        dx, dy = bx - rx, by - ry
        d = math.hypot(dx, dy)
        ux, uy = (1.0, 0.0) if d < 1e-3 else (dx / d, dy / d)
        # stop APPROACH_DIST short of the box, facing it
        return self.make_map_goal(bx - APPROACH_DIST * ux,
                                  by - APPROACH_DIST * uy, math.atan2(uy, ux))

    # Per-mission override of NAV_TIMEOUT_SEC. A class attribute rather than a
    # module constant because how long a drive legitimately takes is a property
    # of the BUILDING, not of the code: hospital_lab's longest leg is about 20 m
    # and 120 s is generous there, while the AWS hospital's collect run is a
    # ~45 m route through the west corridor and times out at 120 s having done
    # nothing wrong. Raising the module constant instead would have made every
    # mission wait four times as long to notice a genuinely stuck robot.
    NAV_TIMEOUT_SEC = NAV_TIMEOUT_SEC

    def clear_costmaps(self, label=''):
        """Throw away both costmaps' accumulated obstacles.

        WHY THIS IS NEEDED BEFORE EVERY GOAL, NOT JUST AFTER A FAILURE.
        The pick parks the robot 0.87 m from a bench and then sweeps the arm
        through the LIDAR's scan plane at 0.4466 m - descend, grasp, break
        contact, lift, carry - at ranges of a few tens of centimetres. Every one
        of those returns is marked into the local costmap as an obstacle, and
        they are marked in a ring that follows the arm around the robot.

        Marking is instant; CLEARING is not. A cell is only cleared when a later
        beam passes THROUGH it and reports free, and the arm has moved away by
        then, so nothing ever ray-traces those cells again. The robot is left
        standing inside a ring of obstacles that do not exist.

        What that looks like from outside is exactly what gets reported as the
        robot "getting confused and spinning": every goal fails to plan, and
        then every recovery refuses to run because the way out is blocked too -

            Running backup -> Collision Ahead - Exiting DriveOnHeading -> failed
            Running spin   -> Collision Ahead - Exiting Spin           -> failed
            Running wait   -> wait completed successfully

        and Nav2 cycles backup/spin/wait for minutes before giving up, with the
        robot sitting at a spot the static map says has 1.30 m of clearance.

        Clearing is cheap and safe: a real obstacle is re-marked on the next
        scan, 0.1 s later, long before the robot has moved anywhere near it.
        """
        log = self.get_logger()
        for name, cli in (('local', self._clear_local), ('global', self._clear_global)):
            if not cli.wait_for_service(timeout_sec=3.0):
                log.warn(f'[nav] {name} costmap clear service unavailable')
                continue
            fut = cli.call_async(ClearEntireCostmap.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.result() is None:
                log.warn(f'[nav] {name} costmap clear did not return')
        log.info(f'[nav] costmaps cleared{" " + label if label else ""}')
        # One scan period, so the real obstacles are back before anyone plans.
        time.sleep(0.3)

    def navigate_to(self, goal_pose, timeout_sec=None, retries=2):
        """Send a NavigateToPose goal and wait for it to actually SUCCEED.
        A goal that ABORTs (e.g. the Nav2 race right after canceling a patrol,
        or a transient plan failure) is retried after a short settle -- treating
        such a completion as 'arrived' would hand off to the visual servo from
        the wrong place."""
        if timeout_sec is None:
            timeout_sec = self.NAV_TIMEOUT_SEC
        log = self.get_logger()
        # Be patient discovering the action server: under heavy sim-startup load
        # DDS discovery of bt_navigator can take well over 10 s.
        if not self.nav_client.wait_for_server(timeout_sec=45.0):
            log.error('[nav] NavigateToPose action server unavailable')
            return False
        # ALREADY THERE? THEN DO NOT ASK NAV2 TO DRIVE THERE.
        #
        # The back-off goal after a pick is the approach pose the robot is still
        # standing on, so it is routinely a goal 0.2 m away. Nav2 handles that
        # badly: the planner returns a path barely longer than a point, the
        # controller cannot extract a heading from it, and the goal checker
        # never fires. Observed on the collect run - the robot sat 0.25 m from
        # the goal while controller_server logged "Passing new path to
        # controller" once a second for the full 300 s timeout, then failed a
        # goal it had been standing on the whole time.
        #
        # Five minutes of fidgeting, then a failure, for a move of 0.2 m. The
        # tolerances used here are the goal checker's own (nav2_params.yaml:
        # xy 0.20, yaw 0.5), so this accepts exactly what Nav2 would accept.
        here = self.robot_in_map()
        if here is not None:
            gq = goal_pose.pose.orientation
            gyaw = math.atan2(2*(gq.w*gq.z + gq.x*gq.y), 1 - 2*(gq.y*gq.y + gq.z*gq.z))
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', self.tf_frame('base_link'), rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0))
                q = tf.transform.rotation
                ryaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            except Exception:
                ryaw = None
            d = math.hypot(here[0] - goal_pose.pose.position.x,
                           here[1] - goal_pose.pose.position.y)
            dyaw = (abs(math.atan2(math.sin(gyaw - ryaw), math.cos(gyaw - ryaw)))
                    if ryaw is not None else 0.0)
            if d <= 0.20 and dyaw <= 0.5:
                log.info(f'[nav] already at the goal ({d:.2f} m, '
                         f'{math.degrees(dyaw):.0f} deg off) -- not driving')
                return True

        for attempt in range(retries + 1):
            # Clear before every attempt. The first one matters because the goal
            # right after a pick starts from inside the phantom ring the arm
            # painted; the retries matter because a goal that just failed has
            # usually spent its recovery cycle adding more of the same.
            self.clear_costmaps(f'before goal attempt {attempt + 1}')
            goal_msg = NavigateToPose.Goal()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose = goal_pose
            log.info(f'[nav] sending Nav2 goal '
                      f'({goal_pose.pose.position.x:.2f},'
                      f'{goal_pose.pose.position.y:.2f})'
                      f'{" (retry)" if attempt else ""}')
            send_fut = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)
            handle = send_fut.result()
            if handle is None or not handle.accepted:
                log.warn('[nav] Nav2 goal rejected -- retrying')
                time.sleep(1.5)
                continue
            result_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_fut, timeout_sec=timeout_sec)
            res = result_fut.result()
            if res is None:
                log.error('[nav] Nav2 goal timed out')
                return False
            if res.status == GoalStatus.STATUS_SUCCEEDED:
                log.info('[nav] Nav2 reached goal')
                return True
            log.warn(f'[nav] Nav2 goal did not succeed (status={res.status}) '
                      f'-- settling and retrying')
            self._stop_base()
            time.sleep(2.0)
        log.error('[nav] Nav2 goal failed after retries')
        return False

    def run_autonomous(self):
        log = self.get_logger()
        log.info('=== NAV AND PICK: START ===')

        det = self.explore_and_find()
        if det is None:
            log.error('Box never detected -- aborting.')
            return
        bx, by, _ = det

        box_map = self.box_in_map(bx, by)
        robot_map = self.robot_in_map()
        if box_map is None or robot_map is None:
            log.error('TF unavailable -- cannot build Nav2 goal.')
            return
        log.info(f'[nav] box in map: ({box_map[0]:.2f},{box_map[1]:.2f})')

        goal = self.compute_approach_goal(box_map, robot_map)
        if not self.navigate_to(goal):
            log.error('Navigation failed -- aborting pick.')
            return

        # Nav2 got us close; visually servo the final short distance precisely,
        # then run the verified scan/grasp/carry/place sequence.
        if self.search_and_approach():
            self.run()
        else:
            log.error('Visual approach failed after navigation -- no pick.')


def main():
    rclpy.init()
    node = NavAndPick()
    ex = rclpy.executors.MultiThreadedExecutor(4)
    ex.add_node(node)

    def task():
        time.sleep(3.0)
        node.run_autonomous()

    t = threading.Thread(target=task, daemon=True)
    t.start()
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
