#!/usr/bin/env python3
"""Mission 2: sort 3 coloured boxes from a table onto 3 matching-coloured columns.

Layout (map frame; map origin = robot's spawn pose):
  * A low table holds 3 boxes in a line -- red, green, blue.
  * 3 columns elsewhere, same x, y spaced 0.3 m, heights 8/12/16 cm -- kept low
    so every placement sits well inside the arm's accurate reach -- each
    coloured to match its box (red/green/blue).

For each box i (in order): drive to the table, claw-pick box i by colour,
drive to column i, centre the BASE on it (front camera, colour blob -- same
mechanism as the box pick), and place the box on top of that column at its
known height. Then drive to a final location and stop.

The arm/wrist orientation never changes across the whole mission: it holds
the zdown ("gripper straight down") pose from pick through carry through
place. Column alignment is done entirely by driving the base, never by
reorienting the arm -- there is no wrist-tilt scan step (an AprilTag-based
version of this used to exist; it needed a one-off wrist tilt to see a tag
lying flat on the column top, which conflicts with keeping the arm/wrist
orientation fixed, so placement uses the column's own body colour instead
of a tag).

Reuses Mission/NavAndPick (navigate_to, claw_approach/claw_pick, grab_below,
_face_box) and PickAndPlace (colour detect, gripper, move_pose).
"""
import math
import time
import threading

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from rclpy.duration import Duration as RclDuration
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from pickplace_arm_bringup.mission import Mission
from pickplace_arm_bringup.pick_and_place import (
    HOME_CONFIG, GRIPPER_X, GRIP_OPEN, BOX_ID, BOX_SIZE, GRASP_LINK,
    FINGER_LINKS, EXPECTED_BOX_Z, GROUND_Z, FRONT_CAM_Z, MAX_REACH_X,
    zdown_quat)
from pickplace_arm_bringup.search_and_pick import (
    APPROACH_LINEAR_GAIN, APPROACH_LINEAR_MAX, APPROACH_LINEAR_MIN,
    APPROACH_ANGULAR_GAIN, APPROACH_ANGULAR_MAX)

# --- layout (map frame) -------------------------------------------------------
#
# Heights here are in the base_link frame, whose origin MOVED when the base
# became a Husky A200: the floor is now at GROUND_Z = -0.13228 instead of
# -0.05. They are written as offsets from GROUND_Z so they stay pinned to real
# heights above the floor rather than drifting with the chassis.
#
# On top of that, poses are now commanded for fr3_hand_tcp (the FINGERTIP
# frame, between the jaws) where they used to be for gripper_base (the gripper
# BODY, which sat ~0.08 m above the held box). So a grasp z is now roughly the
# box CENTRE height, not the body height above it.
# Nav2 parks the base with the box ~1.00 m ahead, and claw_approach then servos
# the last 0.30 m in on the front camera. It must NOT drive closer than this:
# the Husky's bumper is at base_link x=0.4937 and the table's near face is
# 0.125 m in front of the box line, so at 1.00 m standoff there is 0.38 m of
# clearance, while the old 1.45 m (0.85 m standoff) was tuned for a 0.38 m-long
# chassis and is far too tight for this one.
TABLE_APPROACH = (1.30, 0.0, 0.0)      # pose the robot drives to before picking
TABLE_TOP_ABOVE_GROUND = 0.30           # table-top height above the floor
TABLE_Z = GROUND_Z + TABLE_TOP_ABOVE_GROUND          # table-top, base_link frame
# Fingertip z to grasp a box standing on the table: the box centre.
TABLE_GRASP_Z = TABLE_Z + BOX_SIZE / 2.0
# Forward correction applied to the front camera's reading before descending.
#
# MEASURED, not assumed: with the sim paused at the actual grasp stop, the
# camera's detection was compared against ground truth (`gz model -p` for both
# the robot and the box, box centre rotated into base_link). Across all three
# boxes the camera reads SHORT by:
#     red +0.0286   green +0.0300   blue +0.0284      (lateral error <= 3.6 mm)
# i.e. essentially exactly half a box - the camera sees the near FACE and the
# jaws have to go to the CENTRE. So this is BOX_SIZE/2, the same correction the
# ground pick uses, and it is no longer a special case.
#
# It was 0.0 before for a real reason that no longer applies: on the old robot
# the ground-calibrated bias (0.02) OVERSHOT a table box's 0.045 m width and
# shoved it off the table. With a 0.06 box, a 0.076 m jaw opening and a
# correctly measured bias, descending on the centre is both correct and has
# ~8 mm of slack either side.
TABLE_X_OFFSET = 0.030
BOXES = [                               # (colour, box map position) in pick order
    ('red',   (2.30, -0.22)),
    ('green', (2.30,  0.00)),
    ('blue',  (2.30,  0.22)),
]
COLUMNS = [                             # (column id, height, column map x,y)
    (0, 0.30, (-1.0, -0.45)),  # red   -> column 1, 30 cm
    (1, 0.40, (-1.0,  0.00)),  # green -> column 2, 40 cm
    (2, 0.50, (-1.0,  0.45)),  # blue  -> column 3, 50 cm
]
FINAL_POSE = (0.0, -1.8, 0.0)

# Nav2 gets the base to a rough standoff in front of the column; from there
# the base is FINE-centred with the front camera (same colour-blob detector
# used for the table box, pointed at the column body instead), exactly like
# claw_approach centres the base under the gripper for a pick. The arm/wrist
# never moves for this -- it holds zdown throughout -- so there is no ARM
# reachability cliff to guard against (unlike the old AprilTag design, which
# read an actual measured position that could land beyond accurate arm
# reach): the visual servo itself drives to the right standoff distance.
# There IS still a camera-VISIBILITY cliff, though: if Nav2 arrives too far
# off-heading, the column is outside the front camera's FOV and
# approach_column's bounded _face_box correction (capped ~57 deg) cannot
# recover it -- confirmed live (column 1 arrival >57 deg off, zero pixels
# detected, approach aborted). Same reason the old AprilTag design tightened
# Nav2's arrival heading for this goal; still needed here for the same reason.
# Nav2 stops with the column CENTRE this far ahead of base_link.
#
# 1.10, up from 0.42. The old value is an outright collision on this robot: the
# Husky's front bumper is at base_link x=0.4937 and the column is 0.20 deep, so
# its near face would be at 0.42-0.10 = 0.32 - i.e. Nav2 would be asked to park
# with the bumper 0.17 m INSIDE the column. 1.10 leaves 0.51 m of bumper
# clearance, keeps the whole column inside the front camera's cone at arrival
# (near face 0.575 m ahead of the lens, vertical span floor-0.65 m), and leaves
# approach_column a 0.35 m visual servo in to the placement stop.
NAV_STANDOFF = 1.10
# Stop the approach when the column's front-camera reading reaches this x
# (mirrors CLAW_STOP_X for the box pick). The camera only sees the column's
# NEAR FACE, and the columns are now 0.20 deep, so the centre is half that
# (COLUMN_X_OFFSET) further away than the reading.
#
# Stopping with the near face at 0.65 puts the column CENTRE at 0.75 - past the
# bumper (0.4937) with 0.06 m to spare on the near face, comfortably reachable
# (0.67-0.69 m from the arm base for every column height, ~80% of reach), and
# still 0.225 m ahead of the camera lens so the column stays in view at the stop
# rather than falling inside the 0.05 m near clip.
COLUMN_STOP_X = 0.65
COLUMN_Y_TOL = 0.025
COLUMN_X_OFFSET = 0.10

# nav2_params.yaml deliberately loosens yaw_goal_tolerance to 0.5 rad (~29deg)
# to stop the skid-steer oscillating (and tripping its own Spin recovery) at a
# tight tolerance -- but that is too loose for the column approach: an arrival
# heading error beyond approach_column's ~57deg correction cap leaves the
# column outside the front camera's view entirely. Also needed for the TABLE
# approach: measured (ground-truth Gazebo pose vs the front camera's own
# detection at grasp time) a real ~3.5cm lateral grasp-centering error that
# traced back to the base arriving at the table ~10.5deg off the intended
# straight-on heading -- an off-axis camera view biases the HSV-mask
# centroid (the box's visible faces shift with viewing angle), and unlike
# the column-placement overshoot (a FIXED bias, fixable with a constant
# offset) this error scales with arrival angle, so a fixed TABLE_X_OFFSET-
# style correction is the wrong tool; keeping the approach angle small is.
# Tightened for both the table and column approach goals, restored right
# after each, so the looser default (which avoids the oscillation) still
# applies everywhere else (parking, initial patrol).
TIGHT_YAW_TOLERANCE = 0.20
DEFAULT_YAW_TOLERANCE = 0.5


class Mission2(Mission):
    # --- layout (map frame) ---------------------------------------------------
    # These default to the warehouse layout (module constants above). A
    # subclass relocates the SAME mission into a different world just by
    # overriding them -- run_mission_2 below is layout-agnostic and reads only
    # these attributes, never the module globals. (NAV_STANDOFF and the
    # yaw-tolerance constants are tuning, not layout, so they stay shared.)
    LAYOUT_TABLE_APPROACH = TABLE_APPROACH
    LAYOUT_TABLE_GRASP_Z = TABLE_GRASP_Z
    LAYOUT_TABLE_X_OFFSET = TABLE_X_OFFSET
    LAYOUT_BOXES = BOXES
    LAYOUT_COLUMNS = COLUMNS
    LAYOUT_FINAL_POSE = FINAL_POSE
    # Optional camera-frame gate (xmin,xmax,ymin,ymax,zmin,zmax) for the column
    # colour detection (see PickAndPlace._detect). None in the empty warehouse
    # (no clutter to reject); a subclass sets it for a colourful world.
    COLUMN_DETECT_GATE = None
    # How close (front-cam forward reading, m) the base stops from the column
    # before placing, and the max over-column clearance height (m). Defaults
    # tuned for the warehouse; see place_on_column for how they interact with
    # arm reach. A subclass placing on TALL columns stops closer (smaller
    # placement x) so the arm can reach a higher over_z and clear the top.
    COLUMN_STOP_X = COLUMN_STOP_X
    # Cap on the over-column clearance height. On the old 6-DOF arm this was a
    # hard REACH limit (~0.21) and the single tightest constraint in the whole
    # placement - it is why COLUMN_STOP_X_TALL / TALL_COLUMN_H exist at all.
    # The FR3 has no such cliff here: a /compute_ik sweep over every column
    # height (0.30/0.40/0.50) at placement x 0.68-0.80 solved BOTH the lower
    # and the over-column waypoint at every single combination, collision-aware.
    # So this is now just a sanity ceiling well above the tallest column's
    # clearance (0.50 + 0.03 + 0.03 + 0.05 = 0.61 above the floor), not a
    # binding constraint.
    OVER_Z_CEILING = GROUND_Z + 0.80
    # Columns at/above TALL_COLUMN_H are approached CLOSER. Left disabled
    # (threshold above every column height): it existed purely to buy reach
    # headroom the FR3 does not need, and the sweep above confirms the tallest
    # column places fine at the same stop distance as the shortest.
    TALL_COLUMN_H = 99.0
    COLUMN_STOP_X_TALL = COLUMN_STOP_X
    OVER_Z_CEILING_TALL = GROUND_Z + 0.80
    # Forward correction added to the column's detected x before lowering
    # (see place_on_column) -- a subclass overrides this if that world's
    # camera/column rendering carries a different systematic bias than the
    # warehouse's measured ~0.038.
    COLUMN_X_OFFSET = COLUMN_X_OFFSET
    # How badly this world's front camera UNDER-reads the distance to a column
    # (m). 0 in the warehouse (the small COLUMN_X_OFFSET covers it), but in a
    # world where it is large the column is physically out of arm reach at the
    # point the servo declares "centred", and no offset can fix that -- the
    # min() reach cap in place_on_column just clamps the target short and the
    # box is released into thin air. When non-zero, place_on_column closes the
    # gap by creeping the BASE forward (see _creep_forward) instead, because
    # the front camera cannot see the column at that range at all (its near
    # clip is 0.05 m and it sits 0.425 m ahead of base_link, so anything
    # nearer than base x~0.475 falls behind the near plane).
    COLUMN_DEPTH_BIAS = 0.0
    # Where the column should sit (base_link x, m) when the arm actually places,
    # once the creep above has closed the gap. Matches COLUMN_STOP_X +
    # COLUMN_X_OFFSET, i.e. the column centre at the normal stop; verified
    # reachable for all three column heights.
    COLUMN_PLACE_X = COLUMN_STOP_X + COLUMN_X_OFFSET
    # Height of the grasped feature's CENTRE above the payload's own underside.
    # The jaws close on that feature, so the fingertip frame sits exactly this
    # far above whatever surface the payload has to end up standing on - which
    # is what turns a target's top height into a fingertip z, both when picking
    # and when placing.
    #
    # For a cube it is simply half the cube, which is why this used to be
    # written inline as BOX_SIZE / 2.0. It stops being half of anything as soon
    # as the payload is not a cube: the sample rack is grasped by a block on a
    # gantry 0.17 m above its tray, so a placement computed as BOX_SIZE / 2.0
    # would drive the rack 0.14 m THROUGH the shelf it was meant to stand on.
    # Leave the table by navigating back to LAYOUT_TABLE_APPROACH rather than
    # reversing blind. Off by default: the warehouse and hospital_lab picks
    # happen in open rooms where a blind reverse has nothing to hit, and their
    # behaviour should not change. See the back-off in run_mission_2.
    BACK_OFF_VIA_NAV = False

    PAYLOAD_GRIP_HEIGHT = BOX_SIZE / 2.0
    # Where the grasped feature reads (base_link z) with the payload sitting on
    # the FLOOR. _placement_landed compares against this to tell "on the target"
    # from "dropped beside it".
    PAYLOAD_FLOOR_Z = EXPECTED_BOX_Z
    # Colour to look for when verifying a placement. None means "the target's
    # own colour", which is right in the warehouse because each column is
    # painted its box's colour. The hospital run sets it: the dock is green and
    # the rack is red, so the check looks for the rack rather than the dock.
    PLACE_VERIFY_COLOR = None
    # Colour of the placement TARGET, for the servo that centres the base on it.
    # None means "the same colour as the payload" - true in the warehouse, where
    # each column is painted its own box's colour, and the reason this was not a
    # separate value before. It is false anywhere the target is a fixture rather
    # than a colour-matched pillar: the hospital's dock is green and its payload
    # red, and using the payload's colour there finds nothing at all.
    TARGET_COLOR = None
    # Unit vector from the placement target to the standoff the base is sent to,
    # i.e. which SIDE the target is approached from. Yaw is derived from it, so
    # the base always arrives facing the target.
    #
    # (1, 0) reproduces the warehouse exactly: its columns are approached from
    # +x facing -x, because the robot comes from the table on that side. It was
    # hardcoded as col_x + NAV_STANDOFF with yaw = pi, which silently assumed
    # every world approaches from the same side. In the hospital the robot
    # arrives from the corridor to the WEST while the bench body sits east of
    # the dock, so the hardcoded side parked it behind the bench looking at its
    # back panel, with the dock hidden on the far side.
    PLACE_APPROACH_DIR = (1.0, 0.0)

    def __init__(self):
        super().__init__()
        self._set_params_client = self.create_client(
            SetParameters, 'controller_server/set_parameters')
        self.get_logger().info('Mission 2 node ready')

    # --- Nav2 tuning ----------------------------------------------------------
    def _set_yaw_goal_tolerance(self, value, timeout_sec=3.0):
        """Set controller_server's goal-checker yaw tolerance at runtime.
        Returns True on success (best-effort: a failure just leaves the
        previous tolerance in place, which is still a valid, working value)."""
        log = self.get_logger()
        if not self._set_params_client.wait_for_service(timeout_sec=timeout_sec):
            log.warn('[nav] controller_server set_parameters service unavailable')
            return False
        req = SetParameters.Request()
        p = Parameter()
        p.name = 'general_goal_checker.yaw_goal_tolerance'
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
        req.parameters = [p]
        future = self._set_params_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        ok = future.done() and future.result() is not None and future.result().results[0].successful
        if not ok:
            log.warn(f'[nav] failed to set yaw_goal_tolerance={value}')
        return ok

    def _stop_x_for(self, height):
        """Front-cam stop distance for a column of this height. TALL columns
        need the base CLOSER: placing on them needs a higher over-column
        clearance, and the arm's reachable z from the carry branch shrinks as
        it reaches further out (z~0.21 is already marginal at the default
        ~0.38 placement x, which is why the tall column's over-column move
        intermittently failed NO_IK_SOLUTION). Short columns keep the default
        distance, which places them to <5 mm -- approaching them closer was
        measured to make placement WORSE (closer-range detection is less
        accurate), so this is deliberately applied only to tall ones."""
        if height is not None and height >= self.TALL_COLUMN_H:
            return self.COLUMN_STOP_X_TALL
        return self.COLUMN_STOP_X

    def _base_in_odom(self, timeout_sec=0.05):
        """Base position in the ODOM frame. Used only for short relative moves:
        odom is smooth and jump-free (unlike map, which AMCL can snap), which is
        exactly what measuring a ~0.2 m creep needs.

        The timeout is deliberately tiny: this gets polled from inside a
        cmd_vel publish loop, and a long blocking lookup there starves the
        publishing (see _creep_forward). Callers must tolerate None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_frame('odom'), self.tf_frame('base_link'),
                rclpy.time.Time(),
                timeout=RclDuration(seconds=timeout_sec))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'[creep] odom<-base_link TF failed: {e}')
            return None
        t = tf.transform.translation
        return t.x, t.y

    def _creep_forward(self, dist, speed=0.12, timeout_sec=25.0):
        """Drive the base straight forward by `dist` metres, measured with odom
        TF rather than velocity x time. _drive_blind is open-loop (a commanded
        speed a skid-steer base does not actually hold), which is fine for
        "back off far enough to plan" but not for closing a placement gap where
        being 5 cm out drops the box on the floor. Returns True if it covered
        at least 90% of the requested distance.

        cmd_vel must be republished densely: diff_drive_controller stops the
        base if commands stop arriving, so the publish rate -- NOT the odom
        poll rate -- sets whether the robot moves at all. A first version polled
        odom (1 s TF timeout) once per iteration and so managed barely one Twist
        per second; the controller timed out between every pulse and the base
        travelled a measured 0.000 m. Hence: publish every 50 ms unconditionally,
        sample odom only every few cycles with a tiny TF timeout.

        `speed` stays at/above search_and_pick's APPROACH_LINEAR_MIN (0.08), the
        documented floor below which this skid-steer base doesn't reliably
        break static friction."""
        log = self.get_logger()
        if dist <= 0.0:
            return True
        start = self._base_in_odom(timeout_sec=1.0)
        if start is None:
            log.warn('[creep] no odom pose -- skipping creep')
            return False
        twist = Twist()
        twist.linear.x = speed
        deadline = time.time() + timeout_sec
        moved, i = 0.0, 0
        while time.time() < deadline:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
            i += 1
            if i % 4:                     # poll odom ~5 Hz, publish at 20 Hz
                continue
            now = self._base_in_odom()
            if now is not None:
                moved = math.hypot(now[0] - start[0], now[1] - start[1])
                if moved >= dist:
                    break
        self._stop_base()
        time.sleep(0.4)          # let the skid-steer base settle before the arm moves
        final = self._base_in_odom(timeout_sec=1.0)
        if final is not None:
            moved = math.hypot(final[0] - start[0], final[1] - start[1])
        log.info(f'[creep] moved {moved:.3f} m of {dist:.3f} m requested')
        return moved >= dist * 0.9

    def approach_column(self, col_xy, color, timeout_sec=60.0, height=None):
        """Front-camera visual servo that drives the BASE until the `color`
        column sits directly under the gripper's fixed action point --
        mirrors claw_approach for the box pick exactly, just pointed at the
        column's body colour instead of the box. The arm is not touched here
        at all (it holds zdown, carrying the box, the whole time), so unlike
        the old AprilTag scan there is no wrist reorientation and no arm-
        reachability cliff to worry about: centring the BASE on the column is
        what puts it within reach, at a known, fixed local point."""
        log = self.get_logger()
        stop_x = self._stop_x_for(height)
        log.info(f'=== PLACE APPROACH: column (front-cam, base-only, {color}) ===')
        if col_xy is not None:
            self._face_box(col_xy)
        deadline = time.time() + timeout_sec
        twist = Twist()
        lost = 0
        while time.time() < deadline:
            det = self.detect_box_front(timeout_sec=0.25, color=color,
                                        gate=self.COLUMN_DETECT_GATE)
            if det is not None:
                lost = 0
                bx, by, _ = det
                if bx <= stop_x and abs(by) <= COLUMN_Y_TOL:
                    self._stop_base()
                    log.info(f'[place] column centred (front {bx:.2f},{by:+.2f})')
                    return True
                fwd = max(0.0, bx - stop_x)
                # Gate the minimum-speed floor on there being forward distance
                # left to close -- same fix as claw_approach, same failure mode
                # otherwise: once the column is at/inside stop_x but not yet
                # centred laterally, an ungated floor keeps the base crawling
                # into a solid column at 0.08 m/s until the timeout. Turning in
                # place converges on `by` without needing the room.
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
                    log.warn('[place] lost the column -- aborting approach')
                    return False
                # Hold station rather than coasting in on a target that is no
                # longer visible -- same fix, same reason as claw_approach (see
                # the comment there): the old `*= 0.4` kept commanding forward
                # motion through all 12 blind cycles, which here means creeping
                # a box-laden gripper into a solid column.
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            for _ in range(4):
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.03)
        self._stop_base()
        log.warn('[place] column approach timed out')
        return False

    # --- placement ----------------------------------------------------------
    def place_on_column(self, tag_id, height, col_xy, color):
        """Centre the base on the `color` column (front camera, base motion
        only -- the arm/wrist orientation never changes), then lower the held
        box straight down onto its known top height and release. Returns True
        on success."""
        log = self.get_logger()
        if not self.approach_column(col_xy, color, height=height):
            log.warn(f'[place] failed to centre on column {tag_id}')
            return False
        # Take a FRESH reading after the base settles rather than trusting
        # approach_column's last (pre-stop) reading: the skid-steer base has
        # momentum and keeps drifting a little after the stop command, so by
        # the time the arm would act, the column is not quite where it was
        # measured mid-approach -- this silently placed the box short/long of
        # the column top once (landed on the floor beside it, PLACE: DONE
        # notwithstanding). Same fix pattern as grab_below's fresh read before
        # descending onto a box.
        time.sleep(0.4)
        det = self.detect_box_front(timeout_sec=1.5, color=color)
        if det is None:
            log.warn(f'[place] lost sight of column {tag_id} after settling')
            return False
        if self.COLUMN_DEPTH_BIAS > 0.0:
            # This world's front camera under-reads the column distance badly
            # enough that the column is OUT OF ARM REACH at the point the servo
            # calls it centred -- measured live: reading 0.35 m when the column
            # was really ~0.57 m away, against a GRIPPER_X+0.03 = 0.41 m cap.
            # The old code just clamped px to that cap and released, dropping
            # every box on the floor short of its column while still logging
            # "PLACE: DONE". The camera cannot guide the rest of the way (the
            # column is inside its near field, see COLUMN_DEPTH_BIAS), so close
            # the gap open-loop on odom and place at a fixed comfortable reach.
            true_x = det[0] + self.COLUMN_DEPTH_BIAS
            creep = true_x - self.COLUMN_PLACE_X
            log.info(f'[place] column read {det[0]:.2f} m -> true ~{true_x:.2f} m; '
                     f'creeping {creep:.2f} m to place at {self.COLUMN_PLACE_X:.2f}')
            if not self._creep_forward(creep):
                log.warn(f'[place] creep to column {tag_id} fell short -- aborting '
                         f'placement (box still held)')
                return False
            px = self.COLUMN_PLACE_X
            py = det[1]
        else:
            px = min(MAX_REACH_X, det[0] + self.COLUMN_X_OFFSET)
            py = det[1]
        # Fingertip z so the held box's bottom rests on the column top. `height`
        # is measured from the floor, so it has to be lifted into the base_link
        # frame; the box centre then sits half a box above the column top, and
        # the fingertip frame is at the box centre.
        top_z = GROUND_Z + height + self.PAYLOAD_GRIP_HEIGHT
        # The over-column waypoint must clear the column TOP. The held box now
        # hangs ~BOX_SIZE/2 below the fingertip frame, NOT the ~0.08 m it hung
        # below gripper_base, so the clearance term below shrank accordingly.
        #
        # over_z is also bounded by IK reach, and THAT bound is stale: the
        # ceilings below (~0.22 m at x~0.41 rising to ~0.27 at x<=0.34) came
        # from a /compute_ik sweep of the 6-DOF placeholder arm. The FR3 is a
        # different arm, mounted 0.38 m up, with a redundant 7th joint that
        # changes branch behaviour entirely. Re-run that sweep: the whole
        # COLUMN_STOP_X / OVER_Z_CEILING pairing below exists only to work
        # around the OLD arm's reach envelope.
        ceiling = (self.OVER_Z_CEILING_TALL if height >= self.TALL_COLUMN_H
                   else self.OVER_Z_CEILING)
        over_z = min(top_z + BOX_SIZE / 2.0 + 0.04, ceiling)
        log.info(f'=== PLACE: box onto column {tag_id} at ({px:.2f},{py:+.2f}) h={height} ===')
        # Check every move here: a failed move that goes unchecked leaves the
        # arm wherever it happened to stop, and detaching/releasing at THAT
        # unintended spot drops the box off-target while still logging
        # "PLACE: DONE" -- this is exactly what silently dropped the box on
        # the floor next to the column despite a good tag read. If either move
        # fails, abort with the box still attached/grasped (never detach or
        # open the gripper on an unconfirmed position).
        # strict: a failed direct move must never fall back to an unseeded
        # pose plan while holding the box -- that can pick any IK solution,
        # including one that swings the joints all the way around.
        if not self.move_pose(px, py, over_z, 0.0, label='over-column',
                              quat_xyzw=zdown_quat(0.0), strict=True):
            log.warn(f'[place] over-column move failed for column {tag_id} -- '
                     f'aborting placement (box still held)')
            return False
        if not self.move_pose(px, py, top_z, 0.0, cartesian=True,
                              label='lower-onto-column', quat_xyzw=zdown_quat(0.0)):
            log.warn(f'[place] lower-onto-column move failed for column {tag_id} '
                     f'-- aborting placement (box still held)')
            return False
        self.arm.detach_collision_object(BOX_ID)
        time.sleep(0.3)
        self.gripper(GRIP_OPEN, 'release')
        self.move_pose(px, py, over_z, 0.0, cartesian=True, label='retreat',
                       quat_xyzw=zdown_quat(0.0))
        self.arm.remove_collision_object(BOX_ID)
        # ORDER MATTERS: back the BASE off first, and only then return the arm
        # to the claw-ready pose.
        #
        # Doing it the other way round knocks the box straight back off the
        # column. HOME_CONFIG holds the fingertips at 0.50 m above the floor,
        # but the columns are now up to 0.50 m TALL and the placed box sits at
        # 0.50-0.56 m - so coming down to the ready pose while the base is
        # still parked at the column sweeps the open jaws down through the box
        # that was just placed. Measured live: the box ended up on the floor
        # 0.228 m back toward the robot, while every step still reported
        # success. (This could not happen on the old robot: its ready pose was
        # 0.27 m up and its columns only 0.08-0.16 m tall, so the arm always
        # returned well clear above them.)
        #
        # Backing off 0.22 m first puts the column's near face at ~0.85 in
        # base_link, leaving the ready-pose jaws at 0.70 about 0.11 m clear.
        log.info('[place] backing off the column')
        self._drive_blind(-0.22, 3.5)
        self._stop_base()
        self.move_config(HOME_CONFIG, 'gripper-down ready')
        if not self._placement_landed(height, color, tag_id):
            return False
        log.info(f'=== PLACE: DONE (column {tag_id}) ===')
        return True

    def _placement_landed(self, height, color, tag_id):
        """Did the box actually end up ON the column, or on the floor next to
        it? Everything above this point can succeed -- IK solved, cartesian
        lower executed, gripper opened -- while the box drops into thin air
        short of the column, because none of those steps observe the box after
        release. That is not hypothetical: a full 3-box run logged "PLACE:
        DONE" three times with all three boxes measured flat on the floor
        (z=0.0225, i.e. half a 0.045 m cube) and every column untouched.

        The check is the box's own height: detect_box_front already returns the
        blob centroid in base_link, and a box resting on the floor sits at
        EXPECTED_BOX_Z while one on the column top sits a full `height` higher.
        Anything below the midpoint between the two is a failed placement.

        The detection MUST be gated to just above the column top, because each
        column is deliberately painted its box's colour -- so an ungated
        'is there a red blob?' sees the pillar and the box as ONE blob, and the
        pillar wins on area (now 0.20 wide x 0.30-0.50 tall against the box's
        0.06, i.e. far MORE lopsided than before). A placed box would barely
        shift that area-weighted centroid, so the ungated check would fail
        every SUCCESSFUL placement. Gating to a band around where the box
        itself must be leaves the pillar out of the blob entirely.

        The gate is in the CAMERA frame, so the world height of the placed box
        (column top + half a box, above the FLOOR) has to be converted by
        subtracting the camera's own height. That conversion is why the old
        bound was wrong for this robot: it assumed a camera ~0.05 m off the
        floor, and the front camera sits at FRONT_CAM_Z = 0.223 m, so a
        correctly-placed box reads ~0.22 m LOWER in camera z than the old
        expression predicted -- the gate would have excluded every real
        placement. Both bounds are computed from FRONT_CAM_Z, so they follow the
        lens automatically; nothing here needs touching when it moves.

        Visibility at the lower lens height was a worry -- the box sits centred
        on a 0.20 m-deep column, so it is set back 0.07 m behind the column's
        near face, and a pinhole argument said the sight line grazing that
        face's top edge, plus the 73.7 deg vertical cone, should leave the
        0.40 m and 0.50 m columns' boxes invisible from base_link (0.425,
        0.091). MEASURED, THAT IS WRONG: a full 3-box run verified all three,

            red   (0.30 m column)  box z=0.202 vs column top 0.198
            green (0.40 m column)  box z=0.303 vs 0.298
            blue  (0.50 m column)  box z=0.405 vs 0.398

        with ground truth afterwards confirming all three boxes on their
        columns (z = 0.330 / 0.430 / 0.530). The pinhole model was too
        pessimistic: the servo stops nearer 0.73 than the assumed 0.75, and the
        detector needs a blob centroid rather than a fully unoccluded face. Kept
        as a note because if this check ever DOES start failing on the tall
        column while ground truth says the box is placed, the cause is
        geometric visibility, not the gate -- and the fix is to back the base
        off ~0.2 m before looking, not to widen the gate."""
        log = self.get_logger()
        # The gate must START ABOVE THE COLUMN TOP, not merely near the box.
        # Centring it on the box (+/- a few cm) lets the top slice of the
        # pillar into the blob, and since the pillar is 0.20 m wide against the
        # box's 0.06 it instantly dominates the centroid - so the check passes
        # whether or not a box is there. Measured live: a box that had been
        # knocked onto the FLOOR was still "verified at z=0.359", because what
        # the camera actually locked onto was the top 2 cm of the blue column.
        # Starting 5 mm above the top excludes the pillar entirely, so any
        # detection inside the gate really is the box.
        # The gate is centred on where the grasped feature must end up, i.e.
        # PAYLOAD_GRIP_HEIGHT above the target top, and starts above the target
        # top so a same-coloured pillar cannot creep into the blob. For a cube
        # those two coincide (the feature IS the box) and this is the original
        # height+0.005 .. height+BOX_SIZE+0.02 band; for a payload whose colour
        # sits high on a gantry the band follows it up instead of looking at
        # empty air just above the target.
        verify_color = self.PLACE_VERIFY_COLOR or color
        mark_z = height + self.PAYLOAD_GRIP_HEIGHT
        lo = max(height + 0.005, mark_z - BOX_SIZE) - FRONT_CAM_Z
        hi = mark_z + BOX_SIZE - FRONT_CAM_Z
        gate = (0.05, 1.5, -0.6, 0.6, lo, hi)
        det = self.detect_box_front(timeout_sec=2.0, color=verify_color, gate=gate)
        if det is None:
            log.error(f'[place] no {verify_color} payload above target '
                      f'{tag_id}\'s top after release -- it is not on the '
                      f'target. FAILED.')
            return False
        bz = det[2]
        floor_z = self.PAYLOAD_FLOOR_Z
        placed_z = self.PAYLOAD_FLOOR_Z + height
        if bz < (floor_z + placed_z) / 2.0:
            log.error(f'[place] {verify_color} payload is at z={bz:.3f} -- that '
                      f'is floor level (~{floor_z:.3f}), not target {tag_id} top '
                      f'(~{placed_z:.3f}). Placement FAILED.')
            return False
        log.info(f'[place] verified: {verify_color} payload at z={bz:.3f} '
                 f'(target top ~{placed_z:.3f})')
        return True

    # --- full mission -------------------------------------------------------
    def run_mission_2(self):
        log = self.get_logger()
        log.info('=== MISSION 2: START ===')
        if not self.wait_for_localization():
            return

        for (color, box_xy), (tag_id, height, col_xy) in zip(self.LAYOUT_BOXES,
                                                             self.LAYOUT_COLUMNS):
            log.info(f'--- {color} box -> column {tag_id} (h={height}) ---')

            # 1) drive to the table and pick the coloured box off it. Tighten
            # Nav2's arrival heading here too (see TIGHT_YAW_TOLERANCE): an
            # off-axis arrival biases the front camera's HSV-centroid grasp
            # reading, which showed up as a real ~3.5cm lateral grasp-
            # centering error (confirmed via ground-truth Gazebo pose vs the
            # camera's own detection at grasp time) tracing back to a ~10.5deg
            # arrival heading error.
            self._set_yaw_goal_tolerance(TIGHT_YAW_TOLERANCE)
            nav_ok = self.navigate_to(self.make_map_goal(*self.LAYOUT_TABLE_APPROACH))
            self._set_yaw_goal_tolerance(DEFAULT_YAW_TOLERANCE)
            if not nav_ok:
                log.error('Table navigation failed -- aborting.')
                return
            box_map = (box_xy[0], box_xy[1])
            if not self.claw_pick(box_map, color=color,
                                  grasp_z=self.LAYOUT_TABLE_GRASP_Z,
                                  x_offset=self.LAYOUT_TABLE_X_OFFSET):
                log.error(f'Failed to pick the {color} box -- aborting.')
                return

            # Get clear of the table so Nav2 can turn and plan without the
            # bench, right in front of the robot, tripping its collision check.
            #
            # TWO WAYS TO DO THAT, AND THE BLIND ONE IS ONLY SAFE IN AN OPEN
            # ROOM. _drive_blind reverses along whatever heading the robot
            # happens to hold, with no costmap and no look behind. After the
            # visual servo and the grasp that heading is NOT the approach
            # heading any more - measured 55 degrees off it on the AWS hospital
            # collect - so 0.63 m of blind reverse goes somewhere nobody
            # checked. There it drove the chassis onto the bench's own transfer
            # shelf, which stands 0.30 m off the floor against a 0.165 m wheel:
            # the robot ended up 36 degrees nose-up, 0.15 m in the air, in a
            # cell with less clearance than its own footprint. Nav2 then had no
            # valid start pose and aborted every goal for the rest of the run.
            #
            # Navigating back to the approach pose instead costs one Nav2 goal
            # and uses the costmap. That pose is already known good: it is the
            # one the robot drove to, and it is checked for back-off room when
            # it is chosen (see hospital_aws_layout).
            if self.BACK_OFF_VIA_NAV:
                log.info('[mission2] navigating back to the approach pose')
                if not self.navigate_to(
                        self.make_map_goal(*self.LAYOUT_TABLE_APPROACH)):
                    log.warning('[mission2] could not get back to the approach '
                                'pose -- falling back to a blind reverse')
                    self._drive_blind(-0.18, 3.5)
                    self._stop_base()
            else:
                log.info('[mission2] backing off the table')
                self._drive_blind(-0.18, 3.5)
                self._stop_base()

            # 2) drive to a standoff in front of the column (facing it, yaw=pi
            # since the robot approaches from the +x table side), then visually
            # centre the base on it (front camera) and place the box on top.
            # Tighten Nav2's own arrival heading just for this goal:
            # approach_column's heading correction (_face_box) is capped at
            # ~57deg so it can't recover an arbitrarily bad arrival, and a
            # heading error beyond that puts the column outside the front
            # camera's view entirely (confirmed live). Restored right after so
            # the looser default (which avoids skid-steer oscillation) still
            # applies to every other goal.
            target_color = self.TARGET_COLOR or color
            adx, ady = self.PLACE_APPROACH_DIR
            approach = (col_xy[0] + adx * NAV_STANDOFF,
                        col_xy[1] + ady * NAV_STANDOFF,
                        math.atan2(-ady, -adx))
            self._set_yaw_goal_tolerance(TIGHT_YAW_TOLERANCE)
            nav_ok = self.navigate_to(self.make_map_goal(*approach))
            self._set_yaw_goal_tolerance(DEFAULT_YAW_TOLERANCE)
            if not nav_ok:
                log.error(f'Column {tag_id} navigation failed -- aborting.')
                return
            # The box can slip out of the jaws during the drive here (the known
            # intermittent carry slip), and nothing downstream would notice:
            # place_on_column servos onto the column, opens an empty gripper and
            # reports success, and only _placement_landed catches it -- after a
            # pointless full placement. Seen live in this world: box_red ended
            # up on the floor mid-route at world (0.74,-0.24), tipped on its
            # side, while the mission carried on to the column regardless.
            # grasp_is_holding is just a finger-gap read, so this costs nothing.
            if not self.grasp_is_holding():
                log.error(f'{color} box was DROPPED during the carry to column '
                          f'{tag_id} (gripper is empty on arrival) -- aborting.')
                return
            if not self.place_on_column(tag_id, height, col_xy, target_color):
                log.error(f'Failed to place on column {tag_id} -- aborting.')
                return

        log.info('=== MISSION 2: parking ===')
        self.navigate_to(self.make_map_goal(*self.LAYOUT_FINAL_POSE))
        log.info('=== MISSION 2: DONE ===')


class Mission2Tugbot(Mission2):
    """Downloaded OpenRobotics "Tugbot in Warehouse" Fuel world. Reuses the
    plain warehouse layout unchanged (table/columns/final pose): this room's
    open aisle happens to match those coordinates cleanly (verified clear on
    the SLAM map built for it -- maps/tugbot_warehouse.yaml, robot spawn =
    map origin, same convention as the other worlds).

    Unlike the curated pickplace/warehouse worlds, this is a busy downloaded
    scene full of same-coloured clutter (pallet labels, hazard tape, shelf
    trim) that fools the plain HSV blob detector -- confirmed live: an
    ungated 'red' search locked onto something 3+ m out and >1 m up instead
    of the table box. Gate every colour detection (both the box pick and the
    column placement) to dead-ahead/close/low."""
    # Camera-frame gate (X-forward, Y-left, Z-up, metres) that rejects this
    # world's same-coloured clutter. Re-derived for the scaled-up props AND the
    # camera height: the gate's z bounds are relative to the LENS, which sits
    # FRONT_CAM_Z = 0.223 m off the floor, so a world height converts as
    # (height - 0.223). The targets that must stay inside it:
    #   box on the 0.30 m table, centre 0.33  ->  cam z ~ +0.11
    #   column bodies, floor 0.00 to 0.50     ->  cam z -0.22 .. +0.28
    # The band was (-0.36, 0.25) while the lens hung at 0.332; both bounds moved
    # +0.109 with it when the camera was seated on the chassis front panel, so
    # the WORLD slice this admits (-0.03 .. 0.58 m off the floor) is unchanged.
    # Before either, (-0.15, 0.30) was derived when the lens was near floor
    # level; against this camera it would cut off every column below its top few
    # centimetres and the table boxes with it.
    TUGBOT_GATE = (0.05, 2.5, -0.7, 0.7, -0.25, 0.36)
    COLUMN_DETECT_GATE = TUGBOT_GATE
    # Column 0 (short, 8cm) landed a measured 0.19 m short of the column
    # (box at world x=-0.81 vs the column's -1.0) despite "PLACE: DONE" --
    # this world's front-camera depth reading for the column is biased
    # short relative to the warehouse's calibration. A first fix tried
    # stopping the base closer (COLUMN_STOP_X=0.20) to compensate, but that
    # put the column inside the front camera's near-field: live runs lost
    # detection entirely around base x~0.26-0.28 (raw camera-frame x
    # collapsing toward the 0.05 m sensor near-clip) and the approach timed
    # out ("lost the column") before ever reaching the stop distance.
    # TALL_COLUMN_H is never overridden here (stays the base class's 99.0,
    # i.e. no column in this world takes the "tall" branch), so the reach
    # headroom COLUMN_STOP_X was meant to buy in the tall-column case
    # doesn't actually apply -- there is no reach reason to
    # stop this close. Left at the class default (GRIPPER_X - 0.02, same
    # distance verified working in the plain warehouse world).
    #
    # The "short landing" fix also bumped COLUMN_X_OFFSET to 0.10 -- but at
    # the (now restored) default stop distance that pushes px = bx + offset
    # past place_on_column's GRIPPER_X+0.03 reach cap (0.41) on essentially
    # every placement, and 0.41 turned out to be NO_IK_SOLUTION for this
    # column's low over_z (0.19) from the carry-seeded strict IK branch --
    # confirmed live ("[arm] IK seed failed for over-column", box still held,
    # placement aborted). The 0.19 m short-landing measurement was taken
    # under the broken close-stop config, not this one, so it doesn't carry
    # over: left at the class default (0.038), same value verified to place
    # within <5 mm in the plain warehouse world at this same stop distance.
    #
    # DO NOT set COLUMN_DEPTH_BIAS here. A previous session concluded this
    # world's camera under-reads the column distance by 0.22 m, by comparing
    # the released box's true Gazebo pose against the base's AMCL pose -- but
    # AMCL was itself 2.6 m out at the time, so that "bias" was an artefact of
    # mixing a broken frame with a true one. Measured directly instead, with
    # the sim paused and ground truth read from `gz model -p`: base 0.550 m
    # from the column CENTRE, 0.490 m from its near FACE, camera reads 0.493 m
    # (three identical readings). The camera is accurate to 3 mm -- it simply
    # reports the near face, i.e. it reads short by the column's half-width
    # (0.06), exactly what the inherited COLUMN_X_OFFSET (0.038) already
    # compensates for in the plain warehouse world.
    #
    # Setting the bias non-zero here is actively destructive, which is how the
    # phantom was caught: with 0.22 the code turned a correct 0.334 m reading
    # into a believed 0.554 m and creeped the base 0.23 m into a column that
    # had only 0.14 m of clearance. The robot wedged against it (ground truth
    # afterwards: base -0.750, column face -0.940, i.e. bumper flush, zero
    # gap) and the wheels span in place for the creep's full 25 s timeout --
    # 25.4 s x 0.12 m/s = 3.05 m of phantom wheel odometry, which is precisely
    # the 3.0 m odom-vs-truth offset the run ended with, and precisely why
    # AMCL then reported the robot 2.6 m from where it actually stood.

    def detect_box_front(self, timeout_sec=2.0, debug_save=False, color='blue', gate=None):
        if gate is None:
            gate = self.TUGBOT_GATE
        return super().detect_box_front(timeout_sec, debug_save, color, gate)


def _run(node_cls):
    rclpy.init()
    node = node_cls()
    ex = rclpy.executors.MultiThreadedExecutor(4)
    ex.add_node(node)

    def task():
        time.sleep(3.0)
        node.run_mission_2()

    t = threading.Thread(target=task, daemon=True)
    t.start()
    try:
        # rclpy intermittently raises RCLError out of spin() ("wait set index
        # ... out of bounds") from an action-client race under load. Left
        # uncaught it kills the executor and the mission thread hangs forever,
        # so resume spinning instead of dying on it.
        while rclpy.ok():
            try:
                ex.spin()
                break
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                node.get_logger().warn(f'[executor] recovered from spin error: {exc}')
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


def main_tugbot():
    _run(Mission2Tugbot)


if __name__ == '__main__':
    main()
