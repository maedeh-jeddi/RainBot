#!/usr/bin/env python3
"""
Autonomous search-and-pick: drives the mobile base to find the box using the
wrist-mounted camera, then hands off to the existing, already-verified
pick-and-place sequence in pick_and_place.py (unmodified) for the precise
grasp.

Behavior:
  1) Arm holds a forward-and-down "search" pose so the camera can see ahead
     of the robot while it drives.
  2) SEARCH: spin in place, running the same HSV+point-cloud box detector
     used for the close-up pick, until the box is seen. If a full rotation
     finds nothing, drive forward a short distance and spin again.
  3) APPROACH: once seen, steer toward the box (proportional control on
     bearing + distance, both computed directly from detect_box_pose's
     base_link-frame result) until it's within the arm's known reach.
  4) Stop the base, then call the inherited run() -- the exact same
     precise scan+detect+grasp+place sequence pick_and_place.py already
     uses for a stationary robot, untouched.
"""
import math
import time
import threading

import rclpy
from geometry_msgs.msg import Twist

from pickplace_arm_bringup.pick_and_place import (
    PickAndPlace, scan_quat)

# --- unified search/approach vantage point (base_link frame) ------------------
# A single scan pose used for the entire search+approach. Its pitch is chosen
# so its box-detection swath on the ground is CONTINUOUS from ~1.1m out down
# to ~0.45m -- wide enough to spot the box while spinning yet reaching close
# enough to hand off to the pick. This replaces the earlier two-pose scheme
# (a shallow "forward-search" pose + a steep close-up pose): after the arm
# was shortened 30% the two poses' detection ranges no longer overlapped,
# leaving a ~[0.5,0.9]m dead zone the approach fell into and never recovered
# from. Empirically (teleport-and-detect sweep) this pose detects the box
# reliably across [0.45, 1.1]m; the steeper close-up SCAN_POSITION/SCAN_PITCH
# that pick_and_place.run() uses for the final grasp scan covers [0.40,0.50]m,
# so stopping the base with the box near ~0.45m keeps the handoff in range.
SEARCH_POSITION = (0.45, 0.00, 0.30)
# The wrist camera looks ALONG the Franka Hand's approach axis (+z of fr3_hand;
# see camera_joint in the URDF), so it still sees exactly wherever the arm is
# pointed and there is still no mount tilt to add. But the camera axis moved
# 90 deg relative to the old gripper, where it looked along gripper_base's +x -
# perpendicular to the approach. scan_quat/search pitches therefore shift by
# exactly +90 deg to reproduce the same look direction: 53.6 -> 143.6 deg.
# See the fuller explanation on SCAN_PITCH in pick_and_place.py.
#
# SEARCH_POSITION is NOT a carry-over and needs an IK sweep - it is a plausible
# starting point for the FR3 on the Husky, not a verified one, and the
# detection bands quoted above were measured with the old arm and camera.
SEARCH_PITCH = math.radians(143.6)
# Grasp-scan pose used by run() to re-detect the box right before grasping.
# Steeper than the search pose so its detection band ([0.40,0.70]m) reaches
# below the search pose's ~0.45m floor and comfortably covers the ~0.43m stop
# distance -- which is also within the shortened arm's ~0.45m grasp reach.
GRASP_SCAN_POSITION = (0.30, 0.00, 0.42)
GRASP_SCAN_PITCH = math.radians(66.6)

# --- search state machine tuning ------------------------------------------------
# Search rotates in discrete STEPS (rotate a fixed increment, stop, let the
# base settle, then capture a clean point cloud) rather than spinning
# continuously. A fixed 4-wheel skid-steer scrubs and, with the arm extended
# forward in the search pose, orbits slightly about its (forward-shifted)
# COM while rotating; a continuous spin only leaves a narrow window where the
# box is both in-FOV and a fresh cloud lands, which the camera can miss under
# sim load. Stopping to scan makes each detection deterministic.
SPIN_ANGULAR = 0.5              # rad/s during each rotation step
SPIN_STEP_RAD = math.radians(12.0)   # heading increment per scan
SPIN_SETTLE_SEC = 0.6          # let motion damp out before capturing
SPIN_STEPS_PER_REV = int(round(2 * math.pi / SPIN_STEP_RAD)) + 1
BLIND_FORWARD_LINEAR = 0.2      # m/s, when a full spin finds nothing
BLIND_FORWARD_SEC = 2.0

APPROACH_LINEAR_GAIN = 0.5
APPROACH_LINEAR_MAX = 0.35      # m/s
APPROACH_LINEAR_MIN = 0.08      # m/s floor so it keeps closing on the box
APPROACH_ANGULAR_GAIN = 1.4
APPROACH_ANGULAR_MAX = 0.7      # rad/s
STOP_DISTANCE = 0.46            # m; must stay >= the search pose's ~0.45m
                                 # detection floor (stopping closer makes the
                                 # servo lose the box and wander). Box lands
                                 # ~0.45m; the pre-grasp plan there is marginal
                                 # but the cartesian descend still reaches +
                                 # grasps it, so the pick completes.

SEARCH_TIMEOUT_SEC = 180.0


class SearchAndPick(PickAndPlace):
    def __init__(self):
        super().__init__()
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'diff_drive_controller/cmd_vel_unstamped', 10)
        # run() re-detects for the grasp with the steeper grasp-scan pose,
        # whose [0.40,0.70]m band covers the ~0.43m stop distance (the search
        # pose's floor is ~0.45m, and the default steep pose only sees to
        # 0.50m).
        self.scan_position = GRASP_SCAN_POSITION
        self.scan_pitch = GRASP_SCAN_PITCH
        self.get_logger().info('Search-and-pick node ready')

    def _stop_base(self):
        self.cmd_vel_pub.publish(Twist())

    def _drive_blind(self, linear_x, duration_sec):
        twist = Twist()
        twist.linear.x = linear_x
        end = time.time() + duration_sec
        while time.time() < end:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        self._stop_base()

    def _rotate_step(self, angle_rad):
        """Rotate in place by ~angle_rad, then stop and let the base settle so
        the next point-cloud capture is taken while stationary."""
        twist = Twist()
        twist.angular.z = SPIN_ANGULAR if angle_rad >= 0 else -SPIN_ANGULAR
        end = time.time() + abs(angle_rad) / SPIN_ANGULAR
        while time.time() < end:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self._stop_base()
        time.sleep(SPIN_SETTLE_SEC)

    def search_and_approach(self, timeout_sec=SEARCH_TIMEOUT_SEC):
        log = self.get_logger()
        log.info('=== SEARCH: START ===')

        sx, sy, sz = SEARCH_POSITION
        self.move_pose(sx, sy, sz, label='search-scan',
                       quat_xyzw=scan_quat(SEARCH_PITCH))

        twist = Twist()
        deadline = time.time() + timeout_sec
        steps_since_detection = 0

        while time.time() < deadline:
            # Capture while stationary (the base is stopped after each rotate
            # step / approach nudge, so clouds are clean).
            detection = self.detect_box_pose(timeout_sec=1.0)

            if detection is not None:
                steps_since_detection = 0
                bx, by, _bz = detection
                dist = math.hypot(bx, by)
                bearing = math.atan2(by, bx)
                log.info(f'[search] box seen: dist={dist:.2f}m '
                          f'bearing={math.degrees(bearing):.1f}deg')

                if dist < STOP_DISTANCE:
                    self._stop_base()
                    log.info('=== SEARCH: box within reach, stopping ===')
                    return True

                # Approach: nudge toward the box, then stop so the next capture
                # is stationary again. Cap the per-nudge distance (short pulse,
                # gain shrinks it further as the box nears) so the base lands
                # the box in the narrow [0.45,0.50]m window the grasp scan needs
                # instead of overshooting past it.
                nudge_sec = 0.3
                twist.linear.x = min(APPROACH_LINEAR_MAX,
                                      APPROACH_LINEAR_GAIN * dist)
                twist.angular.z = max(-APPROACH_ANGULAR_MAX, min(
                    APPROACH_ANGULAR_MAX, APPROACH_ANGULAR_GAIN * bearing))
                end = time.time() + nudge_sec
                while time.time() < end:
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(0.05)
                self._stop_base()
                time.sleep(SPIN_SETTLE_SEC)
            else:
                # Step-and-scan: rotate a fixed increment, stop, settle, and
                # loop back to capture. After a full revolution with nothing
                # seen, drive forward a short distance and try again.
                self._rotate_step(SPIN_STEP_RAD)
                steps_since_detection += 1
                if steps_since_detection >= SPIN_STEPS_PER_REV:
                    log.info('[search] full rotation, nothing found -- '
                              'driving forward and retrying')
                    self._drive_blind(BLIND_FORWARD_LINEAR, BLIND_FORWARD_SEC)
                    time.sleep(SPIN_SETTLE_SEC)
                    steps_since_detection = 0

        self._stop_base()
        log.error('=== SEARCH: timed out without finding the box ===')
        return False


def main():
    rclpy.init()
    node = SearchAndPick()
    ex = rclpy.executors.MultiThreadedExecutor(4)
    ex.add_node(node)

    def task():
        time.sleep(3.0)
        if node.search_and_approach():
            node.run()
        else:
            node.get_logger().error('Search failed -- not attempting pick.')

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
