#!/usr/bin/env python3
"""Full autonomous warehouse mission.

State machine (localizing on the saved map with AMCL, navigating with Nav2):
  INIT     wait for AMCL localization (map -> base_link TF).
  SEARCH   patrol the map via Nav2 NavigateThroughPoses (drive between
           locations WITHOUT stopping) while the base-mounted FRONT camera
           watches for the box; on a stable detection, cancel the patrol.
  APPROACH transform the box into the map frame, Nav2 to a pose ~0.6 m in
           front of it, then the wrist-camera visual servo for the final
           precise positioning.
  PICK     arm picks the box up and holds it in the carry pose.
  DELIVER  Nav2 to the pre-defined delivery point, carrying the box.
  PLACE    arm puts the box down and returns home.
  PARK     Nav2 to the parking station and stop.

Builds on NavAndPick (navigate_to / box_in_map / make_map_goal /
search_and_approach) and PickAndPlace (detect_box_front / pick_up_box /
place_box_down).
"""
import math
import time
import threading

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
import tf2_ros

from nav2_msgs.action import NavigateThroughPoses

from pickplace_arm_bringup.nav_and_pick import NavAndPick, APPROACH_DIST
from pickplace_arm_bringup.pick_and_place import (
    HOME_CONFIG, scan_quat, GRIPPER_Y)
from pickplace_arm_bringup.search_and_pick import (
    APPROACH_LINEAR_GAIN, APPROACH_LINEAR_MAX, APPROACH_LINEAR_MIN,
    APPROACH_ANGULAR_GAIN, APPROACH_ANGULAR_MAX, SEARCH_POSITION, SEARCH_PITCH,
    GRASP_SCAN_POSITION, GRASP_SCAN_PITCH)

# Two-stage wrist approach distances. The shallow SEARCH pose reliably detects
# the box only down to ~0.45-0.48 m, so the coarse wrist phase hands off to the
# steeper grasp-scan pose EARLY -- at ~0.60 m, while the box is still well
# inside the SEARCH pose's range and already inside the grasp-scan pose's
# [0.40,0.70] band -- instead of driving to the SEARCH pose's detection floor
# (which risks losing the box mid-approach). The grasp-scan phase then creeps
# the base in to STOP_DISTANCE_FINE (~0.40 m), comfortably inside the arm's
# z-down grasp reach where the pre-grasp plan succeeds.
PHASE2_HANDOFF_DIST = 0.60
STOP_DISTANCE_FINE = 0.41

# Claw approach: stop driving when the FRONT camera reads the box's NEAR FACE
# this far ahead. Centre laterally to within CLAW_Y_TOL (the jaws close in y, so
# lateral accuracy matters most).
#
# 0.75, NOT the GRIPPER_X - BOX_SIZE/2 = 0.67 this used to be. That old value
# put the box centre exactly under the gripper-down ready pose, which is a
# tidy-looking invariant and an unusable one: it asks the base to stop 2.8 cm
# short of the point where the front camera CANNOT SEE THE BOX AT ALL.
#
# The lens sits FRONT_CAM_Z = 0.223 m off the floor and the table top is at
# 0.30 m, so the camera looks at a table box from BELOW the surface it stands
# on. All it can ever see is the sliver poking above the table's own near top
# edge, and that sliver shrinks as the base closes in: with the table's near
# face 0.095 m nearer than the box's, the lowest visible point of the box face
# is
#     z(d) = 0.223 + 0.077 * d / (d - 0.095)      (d = lens -> box face, m)
# which reaches the box's top (0.36) at d = 0.217 -- base_link x = 0.642.
#
# MEASURED, not modelled. That expression reproduces the logged detection
# heights across a whole approach to within a millimetre (predicted cam z 0.112
# / 0.114 / 0.117 / 0.121 / 0.126 / 0.132 against 0.112 / 0.114 / 0.117 / 0.122
# / 0.127 / 0.133 logged at box-face distances 0.779 / 0.620 / 0.462 / 0.352 /
# 0.290 / 0.244), and a frame grabbed at the old 0.67 stop is 100% table brown
# with zero red pixels in it.
#
# At 0.75 the box face is still ~47% visible (a solid ~1200-pixel blob, versus
# ~870 and falling at 0.67), with 0.11 m of margin on the blind point and 0.16 m
# of bumper-to-table clearance. The base no longer has to put the box UNDER the
# gripper, because grab_below descends on wherever the box actually is: the
# grasp lands at 0.75 + TABLE_X_OFFSET = 0.78, inside the /compute_ik-verified
# MAX_REACH_X = 0.85 envelope at every working height. Letting the ARM cover
# the last 8 cm is free; making the BASE cover it is not.
#
# Do NOT "fix" this by driving the base the rest of the way blind. That was
# tried (odom-closed-loop creep from 0.88): the mission node's own odom TF read
# stayed frozen at 0.000 m for all three of _creep_forward's 25 s timeouts, so
# it never terminated early, drove the robot into the table and wedged it there
# -- 9.1 m of commanded travel showing up as ~9.6 m of phantom wheel odometry,
# the same signature the column-placement notes in mission_2.py describe.
CLAW_STOP_X = 0.75
CLAW_Y_TOL = 0.02

# Hand-off distance from the front-camera coarse approach to the wrist-camera
# fine approach. 0.9 m: the wide-FOV front camera keeps the (now centred) box in
# view and tracks it down this far, then hands to the wrist SEARCH pose while the
# box is comfortably inside its ~[0.45,1.1] m band -- neither camera loses the
# box, so the chassis never sweeps.
FRONT_HANDOFF_DIST = 0.9

# --- mission targets (map frame; map origin = robot's mapping start pose) -----
# Patrol route the robot sweeps while watching for the box (open lanes in the
# warehouse; each yaw faces the next leg so the front camera looks ahead).
# NOTE: NavigateThroughPoses treats the LAST pose as the goal, so the route must
# NOT end at the robot's current spot (else it "arrives" instantly). It ends
# far from the (0,0) start, and re-patrols reverse direction (see
# search_via_patrol) so a repeat lap also has a distant final goal.
PATROL_WAYPOINTS = [
    (2.5, 0.0), (2.5, -3.5), (-1.0, -4.0), (-3.5, -1.5),
    (-3.5, 1.5), (0.0, 2.0), (2.0, 2.0),
]
DELIVERY_POSE = (-4.0, 2.0, 0.0)     # x, y, yaw -- where the box is dropped off
PARKING_POSE = (4.0, -4.0, 0.0)      # x, y, yaw -- final parking station

FRONT_DETECT_MAX_DIST = 2.5   # only act on front-cam detections within this range
FRONT_DETECT_CONSEC = 2       # consecutive detections before committing
SEARCH_TIMEOUT_SEC = 240.0


class Mission(NavAndPick):
    def __init__(self):
        super().__init__()
        self.tp_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        self.get_logger().info('Mission node ready')

    # --- helpers ------------------------------------------------------------
    def _patrol_poses(self, reverse=False):
        wps = list(reversed(PATROL_WAYPOINTS)) if reverse else list(PATROL_WAYPOINTS)
        poses = []
        prev_yaw = 0.0
        for i, (x, y) in enumerate(wps):
            # face the next leg; the final pose keeps the previous heading
            if i + 1 < len(wps):
                nx, ny = wps[i + 1]
                prev_yaw = math.atan2(ny - y, nx - x)
            poses.append(self.make_map_goal(x, y, prev_yaw))
        return poses

    def wait_for_localization(self, timeout_sec=60.0):
        log = self.get_logger()
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            try:
                self.tf_buffer.lookup_transform(
                    'map', self.tf_frame('base_link'), rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0))
                log.info('[mission] localized (map->base_link available)')
                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                time.sleep(0.5)
        log.error('[mission] no map->base_link TF -- is AMCL running?')
        return False

    def _cancel(self, handle):
        try:
            fut = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        except Exception:
            pass
        self._stop_base()

    # --- SEARCH: patrol (no stopping) while the front camera watches ---------
    def search_via_patrol(self, timeout_sec=SEARCH_TIMEOUT_SEC):
        log = self.get_logger()
        log.info('=== MISSION SEARCH: patrol + front-camera watch ===')
        # tuck the arm compactly (home) so it stays within the footprint and
        # doesn't block the forward view while driving.
        self.move_config(HOME_CONFIG, 'home')

        if not self.tp_client.wait_for_server(timeout_sec=10.0):
            log.error('[mission] NavigateThroughPoses server unavailable')
            return None

        def send_patrol(reverse):
            g = NavigateThroughPoses.Goal()
            g.poses = self._patrol_poses(reverse)
            fut = self.tp_client.send_goal_async(g)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            h = fut.result()
            if h is None or not h.accepted:
                return None, None
            return h, h.get_result_async()

        reverse = False
        handle, result_fut = send_patrol(reverse)
        if handle is None:
            log.error('[mission] patrol goal rejected')
            return None

        consec = 0
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            det = self.detect_box_front(timeout_sec=1.0)
            if det is not None and math.hypot(det[0], det[1]) < FRONT_DETECT_MAX_DIST:
                consec += 1
                log.info(f'[mission] front camera sees box: '
                          f'dist={math.hypot(det[0], det[1]):.2f}m ({consec})')
                if consec >= FRONT_DETECT_CONSEC:
                    log.info('=== MISSION SEARCH: box found, stopping patrol ===')
                    self._cancel(handle)
                    return det
            else:
                consec = 0
            if result_fut.done():   # lap finished without finding -> reverse + re-patrol
                log.info('[mission] patrol lap done, no box -- reversing route')
                reverse = not reverse
                handle, result_fut = send_patrol(reverse)
                if handle is None:
                    return None
                time.sleep(1.0)     # let the new lap start driving before re-checking
        self._cancel(handle)
        log.error('=== MISSION SEARCH: timed out ===')
        return None

    # --- APPROACH: front-cam coarse drive-in, then wrist fine servo ----------
    def _drive_toward(self, dist, bearing, stop_slack):
        """One proportional nudge toward a box seen at (dist, bearing), then stop
        + settle so the next capture is stationary. The stride is ADAPTIVE: far
        from the stop it drives fast for a longer burst (covering ground in few
        cycles); near the stop it slows to short, precise nudges so it doesn't
        overshoot into the box. A floor on the speed guarantees progress across
        the stop threshold instead of stalling just outside it."""
        margin = max(0.0, dist - stop_slack)
        twist = Twist()
        twist.linear.x = min(APPROACH_LINEAR_MAX,
                             max(APPROACH_LINEAR_MIN, APPROACH_LINEAR_GAIN * margin))
        twist.angular.z = max(-APPROACH_ANGULAR_MAX,
                              min(APPROACH_ANGULAR_MAX, APPROACH_ANGULAR_GAIN * bearing))
        # burst length grows with the remaining margin: ~0.25 s creeping up to
        # the stop, up to ~0.6 s when there's a metre to cover.
        burst = min(0.6, max(0.25, 0.9 * margin))
        end = time.time() + burst
        while time.time() < end:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self._stop_base()
        time.sleep(0.15)

    def _servo_phase(self, detect, stop_dist, stop_slack, sweep_cap, deadline):
        """Servo the base toward the box using `detect` (front or wrist) until
        it's within stop_dist. If the box isn't seen, do a bounded left/right
        re-acquire sweep (never a full spin, never a blind forward drive).
        Returns 'reached' / 'lost' / 'timeout'."""
        log = self.get_logger()
        sweep = 0.0
        going_left = True
        while time.time() < deadline:
            det = detect(timeout_sec=1.0)
            if det is not None:
                sweep = 0.0
                bx, by, _ = det
                dist = math.hypot(bx, by)
                bearing = math.atan2(by, bx)
                log.info(f'[approach] box: dist={dist:.2f}m '
                          f'bearing={math.degrees(bearing):.0f}deg')
                if dist < stop_dist:
                    self._stop_base()
                    return 'reached'
                self._drive_toward(dist, bearing, stop_slack)
            else:
                step = 0.15 if going_left else -0.15
                self._rotate_step(step)
                sweep += step
                if going_left and sweep >= sweep_cap:
                    going_left = False
                elif not going_left and sweep <= -sweep_cap:
                    self._stop_base()
                    return 'lost'
        return 'timeout'

    def _face_box(self, box_map):
        """Turn in place to point the base at the box's KNOWN map position so the
        front-camera approach starts with the box centred (Nav2 can arrive up to
        its yaw tolerance off-heading, which otherwise costs a slow re-acquire
        sweep). Done OPEN-LOOP as a single bounded turn from one pose reading --
        a closed feedback loop diverges because the map->base_link yaw lags while
        the base is rotating. The correction is small (Nav2 already roughly faced
        the box) and capped, so a bad estimate can't spin the robot away."""
        log = self.get_logger()
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.tf_frame('base_link'), rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        ryaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        bearing = math.atan2(box_map[1] - t.y, box_map[0] - t.x)
        err = math.atan2(math.sin(bearing - ryaw), math.cos(bearing - ryaw))
        if abs(err) < 0.12:
            return
        err = max(-1.0, min(1.0, err))   # cap the correction at ~57 deg
        log.info(f'[approach] facing box (turn {math.degrees(err):.0f} deg)')
        self._rotate_step(err)

    def servo_to_box(self, box_map=None, timeout_sec=70.0):
        """Final approach after Nav2 leaves the robot near the box. First turn to
        face the box's known map position (so it's centred, no slow sweep), then:
        1) FRONT camera (wide FOV, map-independent) drives the base to
           ~FRONT_HANDOFF_DIST; 2) WRIST SEARCH pose to ~PHASE2_HANDOFF_DIST;
        3) WRIST grasp-scan pose to the grasp stop distance. No phase ever does a
        full 360 spin or a blind forward drive."""
        log = self.get_logger()
        log.info('=== MISSION APPROACH: front-cam coarse + wrist fine ===')
        deadline = time.time() + timeout_sec

        if box_map is not None:
            self._face_box(box_map)

        # Phase 1: front camera. The slack (0.3 m inside the hand-off distance)
        # sets the drive-in speed; _servo_phase still stops at FRONT_HANDOFF_DIST.
        # sweep_cap is small (0.35 rad ~= 20 deg): with the wide-FOV camera and
        # the face-box pre-turn the box stays in view, so this is only a tiny
        # re-acquire wiggle if ever needed -- never a chassis spin.
        r = self._servo_phase(self.detect_box_front, FRONT_HANDOFF_DIST,
                               FRONT_HANDOFF_DIST - 0.3, sweep_cap=0.35,
                               deadline=deadline)
        if r != 'reached':
            log.warn(f'[approach] front-cam phase {r} -- aborting approach')
            return False

        # Phase 2: wrist camera (shallow SEARCH pose, sees [0.45,1.1]) drives
        # the base in to PHASE2_HANDOFF_DIST (~0.60 m) -- an early hand-off that
        # keeps the box well above the SEARCH pose's detection floor so it isn't
        # lost mid-approach.
        self.move_pose(*SEARCH_POSITION, label='search-scan',
                       quat_xyzw=scan_quat(SEARCH_PITCH))
        r = self._servo_phase(self.detect_box_pose, PHASE2_HANDOFF_DIST,
                              PHASE2_HANDOFF_DIST - 0.15, sweep_cap=0.5,
                              deadline=deadline)
        if r != 'reached':
            log.warn(f'[approach] wrist phase {r} -- aborting approach')
            return False

        # Phase 3: steeper grasp-scan pose (sees [0.40,0.70]) creeps the last
        # few cm so the box ends ~0.40 m ahead -- inside the arm's grasp reach,
        # instead of stranded at the ~0.45 m reach edge where the pick misses.
        self.move_pose(*GRASP_SCAN_POSITION, label='grasp-scan',
                       quat_xyzw=scan_quat(GRASP_SCAN_PITCH))
        r = self._servo_phase(self.detect_box_pose, STOP_DISTANCE_FINE,
                              STOP_DISTANCE_FINE - 0.10, sweep_cap=0.4,
                              deadline=deadline)
        if r == 'reached':
            log.info('=== MISSION APPROACH: box within grasp reach ===')
            return True
        log.warn(f'[approach] fine phase {r} -- aborting approach')
        return False

    # --- full mission -------------------------------------------------------
    # --- CLAW approach: keep the gripper down, drive the box under it ---------
    def claw_approach(self, box_map, timeout_sec=60.0, color='blue'):
        """One continuous motion: keep the gripper pointing straight DOWN and use
        the front (chassis) camera to drive the base until the `color` box is
        centred CLAW_STOP_X ahead -- no stop-and-go, no arm reorientation. The
        front camera alone guides the whole approach; the wrist camera isn't
        used (it points down with the gripper). Stops when the box's forward
        reading reaches CLAW_STOP_X (box then within the arm's reach, and still
        visible -- see the constant) and is centred."""
        log = self.get_logger()
        log.info('=== MISSION APPROACH: claw (gripper-down, continuous) ===')
        self.move_config(HOME_CONFIG, 'gripper-down ready')
        if box_map is not None:
            self._face_box(box_map)
        deadline = time.time() + timeout_sec
        twist = Twist()
        lost = 0
        while time.time() < deadline:
            det = self.detect_box_front(timeout_sec=0.25, color=color)
            if det is not None:
                lost = 0
                bx, by, _ = det
                if bx <= CLAW_STOP_X and abs(by) <= CLAW_Y_TOL:
                    self._stop_base()
                    log.info(f'[claw] box within reach (front {bx:.2f},{by:+.2f})')
                    return True
                fwd = max(0.0, bx - CLAW_STOP_X)
                # Only apply the minimum-speed floor while there is still
                # forward distance to close. The floor exists so the skid-steer
                # reliably breaks static friction, but applying it
                # unconditionally means that once the box is at/inside
                # CLAW_STOP_X but still off-centre laterally, the base keeps
                # crawling forward at 0.08 m/s with nothing left to gain --
                # and drives itself into the table.
                #
                # That is exactly how the blue pick failed: the base ended with
                # its bumper hard against the table (detection frozen at
                # (0.627, 0.089) for the whole timeout, byte-identical pixel
                # counts = not moving), because |by|=0.089 never met
                # CLAW_Y_TOL and the loop never stopped pushing.
                #
                # With the floor gated, an x-satisfied/y-unsatisfied state
                # turns IN PLACE instead. That converges: `by` is the box's
                # lateral offset in base_link, so rotating to face the box
                # drives it to ~0 without needing any more room.
                twist.linear.x = (min(APPROACH_LINEAR_MAX,
                                      max(APPROACH_LINEAR_MIN,
                                          APPROACH_LINEAR_GAIN * fwd))
                                  if fwd > 0.0 else 0.0)
                twist.angular.z = max(-APPROACH_ANGULAR_MAX,
                                      min(APPROACH_ANGULAR_MAX,
                                          APPROACH_ANGULAR_GAIN * math.atan2(by, bx)))
            else:
                lost += 1
                if lost > 12:
                    self._stop_base()
                    log.warn('[claw] lost the box -- aborting approach')
                    return False
                # STOP, don't coast. This used to be `twist.linear.x *= 0.4`
                # ("ease off, don't hard-stop"), which still commands forward
                # motion at every one of the 12 lost cycles -- a decaying
                # geometric series, but one that runs while the robot is blind
                # and pointed at whatever it just lost sight of. Measured live
                # on the table pick: the base carried ~0.05 m past its last
                # good reading and ended with its bumper 5 mm off the table
                # (front-camera cloud reading 0.058-0.124 m, entire frame
                # table-brown). Losing the target is precisely when the base
                # must not keep closing on it -- so hold station and let the
                # lost counter decide, exactly as it does for angular.z.
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            # publish continuously (~30 Hz for ~0.12 s) so the base keeps moving
            # smoothly between detections instead of stopping.
            for _ in range(4):
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.03)
        self._stop_base()
        log.warn('[claw] approach timed out')
        return False

    def claw_pick(self, box_map, color='blue', grasp_z=None, x_offset=None):
        """Continuous claw pick of the `color` box: drive it under the gripper
        then descend straight onto it (to grasp_z -- raise for a box on a table).
        `x_offset` corrects the front camera's forward bias at the final descent;
        pass a smaller value for a box on a table (see grab_below).
        Retries the drive-in + grab a few times (re-centring each time) and
        returns False only if it never holds the box."""
        from pickplace_arm_bringup.pick_and_place import GRASP_Z, FRONT_X_OFFSET
        if grasp_z is None:
            grasp_z = GRASP_Z
        if x_offset is None:
            x_offset = FRONT_X_OFFSET
        log = self.get_logger()
        for attempt in range(1, 4):
            log.info(f'--- claw pick attempt {attempt}/3 ({color}) ---')
            if not self.claw_approach(box_map, color=color):
                return False
            if self.grab_below(grasp_z=grasp_z, color=color, x_offset=x_offset):
                return True
            log.warn('[claw] grab missed -- re-centring and retrying')
            self.move_config(HOME_CONFIG, 'gripper-down ready')
            # A miss leaves the base parked at CLAW_STOP_X. That is still a
            # range the camera sees the box from (which is the whole point of
            # where CLAW_STOP_X now sits), but backing off first puts the retry
            # at the same comfortable detection distance the first attempt had,
            # and un-nudges the box's surroundings if the jaws grazed it.
            self._drive_blind(-0.15, 2.0)
        return False

    def run_mission(self):
        log = self.get_logger()
        log.info('=== MISSION: START ===')

        if not self.wait_for_localization():
            return

        det = self.search_via_patrol()
        if det is None:
            log.error('Box never found -- mission aborted.')
            return

        # The patrol detection was taken while driving; let the base settle
        # (the patrol was just canceled) and take a fresh stationary front-cam
        # reading so the map-frame approach goal is accurate.
        self._stop_base()
        time.sleep(2.0)
        fresh = self.detect_box_front(timeout_sec=2.0)
        if fresh is not None:
            det = fresh
        bx, by, _ = det

        # APPROACH: box -> map, Nav2 to ~APPROACH_DIST in front, then wrist servo
        box_map = self.box_in_map(bx, by)
        robot_map = self.robot_in_map()
        if box_map is None or robot_map is None:
            log.error('TF unavailable for approach goal -- aborting.')
            return
        log.info(f'[mission] box in map: ({box_map[0]:.2f},{box_map[1]:.2f})')
        if not self.navigate_to(self.compute_approach_goal(box_map, robot_map)):
            log.error('Approach navigation failed -- aborting.')
            return

        # APPROACH + PICK: gripper-down claw -- drive the box under the gripper in
        # one continuous motion, then descend straight onto it.
        if not self.claw_pick(box_map):
            log.error('Claw pick failed -- aborting.')
            return

        # DELIVER (carry the box to the delivery point)
        log.info('=== MISSION: delivering to drop-off ===')
        if not self.navigate_to(self.make_map_goal(*DELIVERY_POSE)):
            log.error('Delivery navigation failed -- placing where we are.')
        self.place_box_down()

        # PARK
        log.info('=== MISSION: parking ===')
        self.navigate_to(self.make_map_goal(*PARKING_POSE))
        log.info('=== MISSION: DONE ===')


def main():
    rclpy.init()
    node = Mission()
    ex = rclpy.executors.MultiThreadedExecutor(4)
    ex.add_node(node)

    def task():
        time.sleep(3.0)
        node.run_mission()

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
