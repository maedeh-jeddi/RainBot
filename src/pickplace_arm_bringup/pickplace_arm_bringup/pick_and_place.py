#!/usr/bin/env python3
"""
Full pick-and-place for the pickplace_arm in Gazebo Harmonic.

Uses MoveIt (via pymoveit2) for collision-aware motion planning of the arm,
the ros2_control gripper controller for the physical grasp, and the MoveIt
planning scene (attach/detach) so the box is carried correctly and shown in
RViz. The box is a real, physics-enabled model spawned in Gazebo, so a
successful run physically moves it from the pick pose to the place pose.

The box's (x, y) is not known in advance: the arm moves to a fixed scan pose,
the wrist-mounted RGB-D camera locates the box by color in the point cloud,
and that detected position (transformed into base_link via TF) drives the
grasp -- move the box and re-run and it is found and picked again.

Robot: Clearpath Husky A200 base + Franka FR3 (7 DOF) + Franka Hand.

Geometry (all arm poses are for the `fr3_hand_tcp` link, in `base_link`):
  * pick  : box on the ground, (x, y) from detection -> grasp z at the box
            centre, pre-grasp 0.12 m above it
  * place : ground at PLACE_XY

NOTE: the numeric pose constants below have NOT been re-verified for this
robot. They were swept with /compute_ik against a 6-DOF placeholder arm on a
much smaller chassis, and are carried over here only as plausible starting
points. See the comments on each for what specifically needs redoing.
"""
import math
import time
import threading
from threading import Lock

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as RclDuration
from sensor_msgs.msg import PointCloud2, JointState
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform support)

from pymoveit2 import MoveIt2

ARM_JOINTS = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3',
              'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7']
# EMPTY for the FR3, which disables _normalize_roll_config entirely.
#
# The old 6-DOF placeholder arm gave j1/j4/j6 a deliberate +/-2*pi range, so a
# target near one +/-pi limit had an equivalent (same orientation) near the
# other that avoided a useless ~2*pi unwind. NO FR3 joint has that: the widest
# is joint7 at +/-3.05 rad, well under 2*pi, so `angle +/- 2*pi` is always
# outside the limit and there is never an alternative to pick. Leaving the old
# indices here would just generate unreachable targets.
ROLL_JOINT_IDX = ()
ROLL_LIMIT = 2.0 * math.pi - 0.02
# Only finger_joint1 is commanded; fr3_finger_joint2 mimics it (URDF <mimic>),
# so the gripper controller owns exactly one joint.
GRIPPER_JOINTS = ['fr3_finger_joint1']
# The Franka Hand's TCP frame, midway between the fingertips at 0.1034 m along
# the hand's approach axis. This is the frame every arm pose below is for -
# note it is a FINGERTIP frame, whereas the old `gripper_base` was the gripper
# BODY, so grasp/approach heights measured from it are offset relative to the
# old numbers as well as being on a different robot.
GRASP_LINK = 'fr3_hand_tcp'
FINGER_LINKS = ['fr3_leftfinger', 'fr3_rightfinger', 'fr3_hand']

# --- task geometry (base_link frame) -----------------------------------------
#
# Re-derived for the Husky A200 + FR3 with /compute_ik (collision-aware, zdown
# fingertip orientation, seeded from the previous pose in the sequence). All of
# the operating points below were confirmed reachable with margin, and the
# ready -> descend -> lift -> carry chain confirmed branch-flip free (max joint
# step 0.38 rad through the grasp, 0.81 into the carry).
BOX_ID = 'target_box'
# Boxes scaled 0.045 -> 0.06 with the rest of the world (see the models/ SDFs
# and mission_2_tugbot.launch.py). The Franka Hand's jaw gap is 2 x 0.04 =
# 0.08 m, so a 0.06 box still leaves 1 cm of clearance a side when open.
BOX_SIZE = 0.06
# Ground plane in the base_link frame. base_link is the Husky A200 chassis
# origin, which sits wheel_radius - wheel_vertical_offset = 0.1651 - 0.03282
# above the floor. (Was -0.05 on the placeholder base.)
GROUND_Z = -0.13228
# Front camera height above the FLOOR (front_camera_link is at base_link
# z=0.091). Detection gates are expressed in the CAMERA frame, so converting a
# known world height into a gate bound needs this.
#
# WAS 0.20 (= 0.33228 above the floor), when the camera hung unsupported 83 mm
# in front of the chassis and 86 mm above the top of its front panel. It is now
# bolted flat onto that panel, on the A200's own front_bumper_mount frame, so
# the lens dropped to base_link z=0.091 -- see front_camera_joint in
# pickplace_arm.urdf.xacro for the measured panel geometry. Every camera-frame
# gate z band in this package was shifted +0.109 to keep covering the same
# WORLD heights it covered before.
FRONT_CAM_Z = 0.091 - GROUND_Z         # 0.22328 m above the floor
PLACE_XY = (0.70, 0.25)
GRASP_Z = GROUND_Z + BOX_SIZE / 2.0   # fingertip z when grasping a ground box
APPROACH_Z = GRASP_Z + 0.15           # pre-grasp / lift height
# Franka Hand: finger_joint1 is 0.0 fully closed and 0.04 fully open (per-finger
# travel, so the jaw gap is twice this). 0.038 opens to a 0.076 m gap - clear of
# the 0.06 box on both sides without sitting on the hard stop.
GRIP_OPEN = 0.038
GRIP_CLOSED = 0.0
# Where the jaws are parked once the box is WELDED on (see attach_box). 0.0 is
# what the grasp is commanded to, and it has to stay 0.0: the empty-grasp check
# works precisely because closing on air reads ~0.000 while closing on a box is
# stopped by it at ~0.030. But the moment the weld exists the box joins the
# robot's own articulation, box/finger collision switches off, and that still-
# active 0.0 command pulls the jaws the rest of the way -- straight through the
# box, until the fingers disappear inside it. Harmless mechanically (the weld,
# not friction, is carrying the box) but it looks broken.
#
# So after welding, the jaws are sent back out to the box's own half-width less
# 1 mm: the faces land flush on the box with a millimetre of visual bite, no gap
# and no interpenetration. It is cosmetic only -- nothing load-bearing depends
# on where the fingers sit once the weld is there.
GRIP_HOLD = BOX_SIZE / 2.0 - 0.001     # 0.029 for the 0.06 box
# Box models that can be rigidly grasped (one DetachableJoint per colour on the
# robot; the model names are box_red/box_green/box_blue in every world).
BOX_COLORS = ('red', 'green', 'blue')

# --- grasp verification -------------------------------------------------------
# After the jaws close, a finger joint held OPEN by the box reads clearly above
# an empty (fully-closed) grasp. The sense of the reading is unchanged on the
# Franka Hand - finger_joint1 is 0.0 closed and grows as the jaws open - but
# the SCALE is different: each Franka finger travels 0.04 m, so a box of
# half-width 0.0225 should hold finger_joint1 at roughly 0.0225 rather than the
# 0.004-0.005 the old gripper reported. The threshold is set well below that
# expected value and well above zero.
#
# With the 0.06 box, a holding finger_joint1 should sit near 0.030 (half the
# box) against ~0.000 on air, so this threshold has huge margin either side.
FINGER_HELD_MIN = 0.015
# Grasp attempts before giving up: a fresh scan + descend each time, so a box
# nudged by a missed first attempt is re-located and re-grasped instead of the
# robot silently carrying nothing.
MAX_GRASP_ATTEMPTS = 3

# Compact "carry" pose: box held over the base so it rides stably while the
# mobile base drives to the delivery point.
#
# Re-derived. Two clearances drive it, both re-checked for this robot:
#   * LIDAR: modelled as a real SICK TiM5xx standing on the top plate, its
#     laser centre is 0.4466 m above the floor. The carried box rides at
#     0.65 m, so its underside (0.62) clears the scan plane by ~0.17 m and
#     cannot be self-detected as an obstacle dead ahead (the failure the old
#     0.18->0.30 tuning chain was chasing).
#   * FRONT CAMERA: at base_link (0.425, 0.091). The box at x=0.50 is now
#     AHEAD of the lens rather than behind it, but it still cannot occlude the
#     column search, for a stronger reason than before: it rides 0.40 m above
#     the lens at only 0.045 m of forward separation, i.e. ~84 deg up, where
#     the camera's 73.7 deg vertical cone spans just z 0.057..0.125 in
#     base_link. The whole box sits far outside the frustum.
# Seeded IK from the post-lift state reaches it in one smooth step (0.81 rad,
# no branch flip), so the strict carry move has a same-branch solution.
CARRY_POSITION = (0.50, 0.00, GROUND_Z + 0.65)

# Neutral / "ready" claw configuration (joint angles j1..j7): the gripper points
# STRAIGHT DOWN with the fingertips at (GRIPPER_X, 0, 0.50 above the floor).
#
# Derived with /compute_ik (collision-aware, zdown fingertip). Franka's own
# `ready` pose CANNOT be used for this: it puts the fingertips at x=0.387,
# which on a Husky is directly over the robot's own top plate (the chassis
# spans x +/-0.494) and BEHIND the front camera at x=0.509 - so the gripper
# could not descend without hitting the robot, and the camera could not see
# what it was descending onto. The claw point has to sit beyond the front
# bumper, which is what GRIPPER_X below does.
HOME_CONFIG = [0.0, 0.5012, 0.0, -1.9509, 0.0, 2.452, 0.7854]

# Claw geometry: where the fingertips sit (base_link) in the HOME_CONFIG pose,
# and the grasp/lift heights. The base positions the box under GRIPPER_X/Y,
# then the gripper descends straight down.
#
# GRIPPER_X = 0.70 was chosen against three hard constraints, not tuned:
#   1. CLEAR OF THE ROBOT. The Husky chassis ends at x=0.4937, so anything the
#      gripper descends onto must be beyond that or the arm drives into its own
#      top plate. 0.70 leaves 0.21 m of clearance.
#   2. IN FRONT OF THE CAMERA. front_camera_link is at x=0.425; the claw point
#      must be ahead of it to be seen at all. 0.70 puts the target 0.275 m ahead
#      of the lens - past its 0.05 m near clip, and at that range its 73.7 deg
#      vertical FOV spans floor heights 0.02..0.43, which brackets the 0.30 m
#      table top and the boxes standing on it (top at 0.36). The lower lens
#      buys MORE range here, not less: the old mount saw 0.19..0.48 at 0.19 m.
#   3. COMFORTABLY REACHABLE. 0.62 m from the arm base, i.e. 73% of the FR3's
#      0.855 m reach, so it is nowhere near the singular edge.
# READY_Z holds the fingertips 0.50 m up: high enough to clear the 0.36 m box
# tops while driving, low enough that the descent is short.
GRIPPER_X = 0.70
GRIPPER_Y = 0.0
READY_Z = GROUND_Z + 0.50
# Reach check, from the URDF: fr3_link0 sits 0.3837 m above the floor, so at
# the FR3's full 0.855 m reach the floor is reachable out to a 0.7641 m
# horizontal radius about the arm base - x <= 0.844 in base_link, i.e. 0.35 m
# beyond the Husky's front bumper (chassis front face is at x=0.4937).
#
# The front camera sees a box's NEAR FACE, not its centre, so the detected x is
# half a box short of where the jaws must go. For the 0.06 box that is 0.03.
FRONT_X_OFFSET = BOX_SIZE / 2.0
# Hard cap on how far forward a measured target may be acted on, so a bad
# detection can never command an unreachable pose. Verified with /compute_ik:
# zdown fingertip poses solve out to x=0.85 at every working height (0.33-0.61
# above the floor). Was GRIPPER_X + 0.03, which on the old arm WAS the reach
# edge; here that would clamp legitimate column placements (0.75) short.
MAX_REACH_X = 0.85

# Expected box centroid height in base_link frame (ground plane, see add_box):
# used only as a sanity check against the detected z, not as the commanded z.
EXPECTED_BOX_Z = GROUND_Z + BOX_SIZE / 2.0

# --- perception ---------------------------------------------------------------
# Fixed vantage point the arm moves to before scanning for the box, so the
# eye-in-hand camera's frustum covers the ground ahead of the robot.
#
# THE WRIST CAMERA'S AXIS MOVED BY 90 DEGREES. On the old gripper the camera
# looked along gripper_base's +x, which is PERPENDICULAR to the approach
# direction (+z) - hence the old note that at zdown the camera saw only
# horizontally and scanning needed a pitched pose. On the Franka Hand the
# camera is mounted looking along the hand's +z, i.e. ALONG the approach axis
# (see camera_joint in the URDF), so it now sees exactly where the gripper is
# going. That is the better arrangement, but it inverts what `pitch` means.
#
# SCAN_PITCH is therefore the old 83.6 deg plus exactly the 90 deg the camera
# axis rotated by. scan_quat(pitch) rotates about y, mapping the camera axis
# to (cos p, 0, -sin p) before and (sin p, 0, cos p) now; at 173.6 deg that
# gives (0.1115, 0, -0.9938), the SAME look direction the old 83.6 deg
# produced - near straight down, tilted slightly forward.
#
# SCAN_POSITION holds the fingertips out beyond the bumper and high, so the
# wrist camera (0.0634 m behind hand_tcp along the approach axis) looks down
# over the workspace from ~0.75 m. Only the standalone pick_and_place demo uses
# this; mission_2 never scans with the wrist, it drives the base under the claw.
SCAN_POSITION = (0.60, 0.00, GROUND_Z + 0.70)
SCAN_PITCH = math.radians(173.6)

# HSV bounds (OpenCV H 0-180) for each box colour. Red wraps around H=0, so it
# needs TWO ranges. Each entry is a list of (lower, upper) HSV tuples; a pixel
# matches the colour if it falls in ANY of the ranges.
COLOR_HSV = {
    'blue':  [((95, 120, 60), (115, 255, 255))],
    'green': [((35, 80, 40), (85, 255, 255))],
    'red':   [((0, 100, 50), (10, 255, 255)), ((170, 100, 50), (180, 255, 255))],
}
# Backward-compatible default (the original single blue box).
HSV_LOWER = COLOR_HSV['blue'][0][0]
HSV_UPPER = COLOR_HSV['blue'][0][1]
MIN_VALID_PIXELS = 30


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def zdown_quat(yaw):
    """Gripper pointing straight down, yawed about world z (xyzw)."""
    cz, sz = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return qmul((0.0, 0.0, sz, cz), (1.0, 0.0, 0.0, 0.0))


def scan_quat(pitch, yaw=0.0):
    """Gripper tilted forward-down by `pitch` from horizontal, then yawed
    about world z (xyzw). Distinct from zdown_quat: at pitch=0 the gripper
    (and the camera mounted on it) points straight out horizontally."""
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    yaw_q = (0.0, 0.0, sy, cy)
    pitch_q = (0.0, sp, 0.0, cp)
    return qmul(yaw_q, pitch_q)


class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        cbg = ReentrantCallbackGroup()

        self.arm = MoveIt2(
            node=self, joint_names=ARM_JOINTS, base_link_name='base_link',
            end_effector_name=GRASP_LINK, group_name='arm', callback_group=cbg)
        # Kept slow (0.20, was 0.30): the box is held by friction, and the
        # slip that occasionally dropped it happened DYNAMICALLY during the
        # lift/carry, not statically -- a gentler carry keeps the inertial
        # load on the friction grip below the point where the box breaks free
        # (especially in a heavier world where the physics step is coarser
        # under load). Pairs with the raised finger/box friction.
        self.arm.max_velocity = 0.20
        self.arm.max_acceleration = 0.20

        # Scan pose used by run() to locate the box before grasping. Kept as
        # instance attributes so subclasses (e.g. the mobile search-and-pick)
        # can substitute a pose whose detection range matches where they stop
        # the base, without duplicating run().
        self.scan_position = SCAN_POSITION
        self.scan_pitch = SCAN_PITCH

        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        # --- perception: point cloud subscriptions + TF ---
        # Wrist (eye-in-hand) RGB-D for the precise grasp scan, and the
        # base-mounted front RGB-D for detecting the box while driving.
        self._cloud_lock = Lock()
        self._latest_cloud = None
        self.create_subscription(
            PointCloud2, '/camera/points', self._cloud_cb, 1,
            callback_group=cbg)
        self._front_lock = Lock()
        self._front_cloud = None
        self.create_subscription(
            PointCloud2, '/front_camera/points', self._front_cloud_cb, 1,
            callback_group=cbg)

        # Latest joint positions -- used to VERIFY a grasp actually holds the
        # box (finger positions) and to seed/normalise IK so arm moves take the
        # nearest, simplest joint path (current arm config).
        self._joint_pos = {}
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10,
            callback_group=cbg)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Rigid grasp (see the DetachableJoint plugins in
        # pickplace_arm.gazebo.xacro): welding the box to the gripper on a
        # verified grasp, because a friction hold slipped mid-carry on every
        # Tugbot-warehouse run. These publish through the ros_gz bridge.
        self._attach_pubs, self._detach_pubs = {}, {}
        for c in BOX_COLORS:
            self._attach_pubs[c] = self.create_publisher(Empty, f'/box_{c}/attach', 10)
            self._detach_pubs[c] = self.create_publisher(Empty, f'/box_{c}/detach', 10)
        self._attached_color = None
        # The plugins weld each box the moment it spawns -- metres away, with
        # the robot about to drive off and drag it. Break those welds before
        # anything moves. Safe when nothing is attached (the plugin just
        # reports it is not attached).
        self.detach_box(log_label='startup')

        self.get_logger().info('Pick-and-place node ready')

    # --- rigid grasp ---------------------------------------------------------
    def _publish_box_cmd(self, pub, wait_for_bridge=True):
        """Publish an Empty to a gz DetachableJoint topic. Waits for the bridge
        to subscribe first: these are one-shot commands with no retry loop and
        no acknowledgement, so a message sent before discovery completes is
        simply lost -- and a lost attach means the box rides on friction alone
        again, which is the exact failure this replaces."""
        deadline = time.time() + 5.0
        while wait_for_bridge and pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.1)
        for _ in range(3):
            pub.publish(Empty())
            time.sleep(0.05)

    def attach_box(self, color):
        """Weld the `color` box to the gripper. Call ONLY after the finger-gap
        check confirms the box is really between the jaws -- the joint is
        created at the current relative pose, so attaching without a real grasp
        would pin the box wherever it happens to be."""
        if color not in self._attach_pubs:
            self.get_logger().warn(f'[attach] no attach topic for colour {color}')
            return
        # Settle the jaws onto the box's faces BEFORE the weld, not after.
        # Welding is the one and only thing that turns box/finger collision off,
        # so the still-active GRIP_CLOSED=0.0 command is only ever free to drive
        # the fingers through the box in the window between the weld and this
        # move. Retiring that command first closes the window entirely: the box
        # is still holding the jaws open at ~0.030 here, and GRIP_HOLD (0.029)
        # asks for marginally less than that, so nothing visibly moves and the
        # grasp is never loosened -- but once collision goes away there is no
        # longer a 0.0 command left to plunge them inward. Doing it after works
        # functionally, the weld having already made the carry rigid, but shows
        # a full close-then-reopen cycle on screen.
        #
        # This lives HERE rather than at the call site for the same reason
        # detach is hooked on gripper-open: welding is the one and only place
        # that has to deal with the collision switch-off, so hooking it here
        # means a future call site cannot forget.
        #
        # release=False because this is an opening motion by position that must
        # NOT be read as letting go.
        self.gripper(GRIP_HOLD, 'settle jaws onto the box faces', release=False)
        self._publish_box_cmd(self._attach_pubs[color])
        self._attached_color = color
        self.get_logger().info(f'[attach] {color} box welded to the gripper')

    def detach_box(self, color=None, log_label=''):
        """Release the weld. With no colour, releases every box (used at startup
        and on any gripper open, where the safe move is to make sure nothing is
        left attached)."""
        colors = [color] if color else list(self._detach_pubs)
        for c in colors:
            # Wait per colour, not just for the first: each publisher matches
            # the bridge independently, and a detach dropped for one box leaves
            # THAT box welded to the gripper from spawn -- the robot then tows
            # it off the table on the way to the pick.
            self._publish_box_cmd(self._detach_pubs[c])
        if self._attached_color or log_label:
            self.get_logger().info(
                f'[attach] released {color or "all boxes"} {log_label}'.strip())
        self._attached_color = None

    def _cloud_cb(self, msg):
        with self._cloud_lock:
            self._latest_cloud = msg

    def _front_cloud_cb(self, msg):
        with self._front_lock:
            self._front_cloud = msg

    def _joint_state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._joint_pos[name] = pos

    def grasp_is_holding(self):
        """True if a box is currently pinched between the jaws. A finger joint
        held open by the box reads well above an empty (closed-on-air) grasp;
        use the wider-open of the two fingers so an off-centre box (which stops
        only one finger) still counts as held."""
        # A welded box is held by a joint, not by the fingers, and the finger
        # gap stops meaning anything the moment the weld exists: attaching puts
        # the box in the robot's own articulation, which disables box/finger
        # collision, so the jaws close straight THROUGH the box and the gap
        # reads empty. Observed live -- the first welded run reported "box
        # slipped during lift/carry" seconds after a confirmed grasp, with the
        # box rigidly attached the whole time. The weld is the stronger
        # guarantee anyway: it cannot slip.
        if self._attached_color:
            return True
        if not self._joint_pos:
            return False
        gap = max(self._joint_pos.get(j, 0.0) for j in GRIPPER_JOINTS)
        return gap > FINGER_HELD_MIN


    # --- primitives ----------------------------------------------------------
    def move_pose(self, x, y, z, yaw=0.0, cartesian=False, label='',
                  quat_xyzw=None, strict=False):
        self.get_logger().info(
            f'[arm] -> ({x:.2f},{y:.2f},{z:.2f}) yaw={yaw:.2f} '
            f'{"cartesian " if cartesian else ""}{label}')
        if quat_xyzw is None:
            quat_xyzw = zdown_quat(yaw)
        if cartesian:
            self.arm.move_to_pose(position=(x, y, z), quat_xyzw=quat_xyzw,
                                  cartesian=True, cartesian_fraction_threshold=0.0)
            ok = self.arm.wait_until_executed()
        else:
            ok = self._move_pose_direct(x, y, z, quat_xyzw, label, strict)
        if not ok:
            self.get_logger().warn(f'[arm] motion failed: {label}')
        time.sleep(0.5)
        return ok

    def _move_pose_direct(self, x, y, z, quat_xyzw, label, strict=False):
        """Non-cartesian pose move that takes the SIMPLE, direct path: solve IK
        seeded from the CURRENT joint state (so the nearest configuration is
        chosen -- no elbow/wrist flip), then plan a short joint-space move to
        it. A bare pose goal lets MoveIt pick any IK solution, which is often a
        far one that swings the joints all the way around -- so with
        `strict=True` (used for every move while the arm is holding a box) we
        never fall back to it: a failed seed/joint-move just fails the whole
        call, rather than risking exactly the kind of big uncontrolled swing
        that could visibly rotate the arm and shake the box loose."""
        sol = self.arm.compute_ik(position=(x, y, z), quat_xyzw=quat_xyzw)
        cfg = self._extract_arm_config(sol) if sol is not None else None
        if cfg is not None:
            cfg = self._normalize_roll_config(cfg)
            self.arm.move_to_configuration(cfg)
            if self.arm.wait_until_executed():
                return True
            self.get_logger().warn(f'[arm] direct joint move failed for {label}'
                                   + ('' if strict else '; trying pose plan'))
        else:
            self.get_logger().warn(f'[arm] IK seed failed for {label}'
                                   + ('' if strict else '; using pose plan'))
        if strict:
            return False
        self.arm.move_to_pose(position=(x, y, z), quat_xyzw=quat_xyzw,
                              cartesian=False, cartesian_fraction_threshold=0.0)
        return self.arm.wait_until_executed()

    @staticmethod
    def _extract_arm_config(joint_state):
        """Pull the ARM_JOINTS positions (in order) out of an IK JointState."""
        try:
            return [joint_state.position[joint_state.name.index(j)]
                    for j in ARM_JOINTS]
        except (ValueError, IndexError):
            return None

    def _current_arm_config(self):
        if not all(j in self._joint_pos for j in ARM_JOINTS):
            return None
        return [self._joint_pos[j] for j in ARM_JOINTS]

    def _normalize_roll_config(self, config):
        """For each ROLL joint, replace the target angle with the equivalent
        (+/- 2*pi) that is CLOSEST to the current angle while staying inside the
        joint's +/-2*pi limit. This removes the useless ~2*pi unwind when the
        goal is near one pi and the arm is near the other (same orientation)."""
        cur = self._current_arm_config()
        if cur is None:
            return config
        config = list(config)
        for i in ROLL_JOINT_IDX:
            best = config[i]
            for alt in (config[i] - 2.0 * math.pi, config[i] + 2.0 * math.pi):
                if -ROLL_LIMIT <= alt <= ROLL_LIMIT and abs(alt - cur[i]) < abs(best - cur[i]):
                    best = alt
            config[i] = best
        return config

    def move_config(self, config, label=''):
        """Move to an explicit joint configuration (a direct joint-space plan).
        Roll-joint targets are normalized to the nearest equivalent so the arm
        never unwinds ~2*pi to reach the same orientation."""
        config = self._normalize_roll_config(config)
        self.get_logger().info(f'[arm] -> configuration {label}')
        self.arm.move_to_configuration(config)
        ok = self.arm.wait_until_executed()
        if not ok:
            self.get_logger().warn(f'[arm] motion failed: config {label}')
        time.sleep(0.5)
        return ok

    def gripper(self, pos, label='', release=None):
        """Command the jaws to `pos`.

        `release` overrides the automatic detach-on-open below. Left None it
        keeps the rule every existing call site relies on -- opening at all,
        while welded, releases the box. Pass False for a move that opens the
        jaws WITHOUT meaning "let go": the only such move is settling them onto
        the box faces just before the weld (see attach_box), which is an opening
        motion by position but the exact opposite of a release by intent.
        """
        self.get_logger().info(f'[gripper] -> {pos} {label}')
        if release is None:
            release = pos > GRIP_CLOSED and self._attached_color
        m = JointTrajectory()
        m.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        # One entry per commanded joint. The Franka Hand exposes a single
        # commandable joint (finger_joint2 mimics finger_joint1), where the old
        # gripper had two, so this is sized off GRIPPER_JOINTS rather than
        # hardcoded.
        pt.positions = [float(pos)] * len(GRIPPER_JOINTS)
        pt.time_from_start = Duration(sec=1)
        m.points = [pt]
        for _ in range(3):
            self.gripper_pub.publish(m)
            time.sleep(0.4)
        time.sleep(1.0)
        # Release the weld AFTER the jaws have finished opening, never before.
        # Opening is the one and only release point, so hooking it here covers
        # every call site (place, release-after-failed-carry, the open before a
        # fresh grab) instead of relying on each to remember. Order matters: a
        # welded box shares the robot's articulation, so box/finger collision is
        # off. Detaching first would risk handing the physics engine two
        # interpenetrating bodies and letting it fire the box out sideways;
        # opening first clears the fingers (GRIP_OPEN 0.038 > the box's 0.030
        # half-width) so the box simply drops the last millimetre onto the
        # column when the weld goes. (The jaws no longer sit deep inside the box
        # by this point -- attach_box settles them onto its faces -- but they
        # are still touching it, so the ordering still matters.)
        if release:
            self.detach_box(log_label=f'on gripper open ({label})')

    def add_box(self, xy, z_center=None):
        if z_center is None:
            z_center = GROUND_Z + BOX_SIZE / 2.0  # ground box in base_link frame
        self.arm.add_collision_box(
            id=BOX_ID, size=(BOX_SIZE, BOX_SIZE, BOX_SIZE),
            position=(xy[0], xy[1], z_center), quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            frame_id='base_link')
        time.sleep(0.5)

    # --- perception ------------------------------------------------------------
    def detect_box_pose(self, timeout_sec=5.0, debug_save=False, color='blue'):
        """Wrist (eye-in-hand) detection: move must already be at the scan pose.
        Returns the box centroid (x, y, z) in base_link, or None."""
        return self._detect('wrist', timeout_sec, debug_save, color)

    def detect_box_front(self, timeout_sec=2.0, debug_save=False, color='blue',
                         gate=None):
        """Base-mounted front camera detection (used while driving). Returns the
        `color` box centroid (x, y, z) in base_link, or None. `gate` (see
        _detect) optionally restricts detection to a camera-frame box, which
        rejects same-coloured background clutter."""
        return self._detect('front', timeout_sec, debug_save, color, gate)

    def _detect(self, source, timeout_sec, debug_save=False, color='blue',
                gate=None):
        """Waits for a fresh point cloud from the given RGB-D source, HSV-segments
        the blue box, and returns its centroid (x, y, z) in base_link, or None.
        `source` is 'wrist' (/camera/points, camera_link) or 'front'
        (/front_camera/points, front_camera_link).

        `gate`, if given, is (xmin, xmax, ymin, ymax, zmin, zmax) in the CAMERA
        frame (X-forward, Y-left, Z-up) -- only coloured pixels whose 3D point
        falls inside this box are counted. In a plain/empty world this is
        unnecessary, but in a cluttered, colourful world (e.g. the Ionic
        restaurant, whose walls/beams/pillars are the SAME navy blue as the
        blue box/column) the raw HSV blob is dominated by background
        architecture; gating to where the target is actually expected (dead
        ahead, low, and near, for a column the robot has driven up to) is what
        lets the colour servo lock onto the target instead of the walls."""
        log = self.get_logger()
        if source == 'front':
            lock, cloud_frame = self._front_lock, 'front_camera_link'
        else:
            lock, cloud_frame = self._cloud_lock, 'camera_link'
        with lock:
            if source == 'front':
                self._front_cloud = None
            else:
                self._latest_cloud = None

        deadline = time.time() + timeout_sec
        cloud = None
        while time.time() < deadline:
            # actively pump this node's own callbacks while waiting (same
            # idiom pymoveit2's wait_until_executed uses) instead of relying
            # solely on the background executor thread to service us.
            rclpy.spin_once(self, timeout_sec=0.2)
            with lock:
                cloud = self._front_cloud if source == 'front' else self._latest_cloud
            if cloud is not None:
                break
        if cloud is None:
            log.error('[detect] no point cloud received before timeout')
            return None

        h, w = cloud.height, cloud.width
        if h <= 1:
            log.error('[detect] point cloud is not organized (height <= 1)')
            return None

        # read_points ALREADY returns a numpy structured array (Humble onwards).
        # Wrapping it in `list()` first iterated it into 307,200 np.void rows and
        # rebuilt the array from them, which cost a MEASURED 0.86 s per frame on
        # this 640x480 cloud against 0.0001 s for the array itself -- bit-identical
        # output, ~7000x. That single line was what capped every visual servo in
        # this package at ~1 Hz: claw_approach's loop is detect + 4 x 0.03 s of
        # cmd_vel publishing, so it should go round at ~5 Hz (camera-limited at
        # 15 Hz), but it measured ~0.95 s per iteration. At the
        # APPROACH_LINEAR_MIN 0.08 m/s floor that is ~0.08 m of base travel
        # between consecutive readings, i.e. the servo could only ever stop to
        # within ~8 cm of its target -- far too coarse for the table pick, whose
        # usable stopping window is a few cm wide (see CLAW_STOP_X in
        # mission.py). At 7 Hz it is ~1 cm per cycle instead.
        pts = point_cloud2.read_points(
            cloud, field_names=('x', 'y', 'z', 'rgb'), skip_nans=False)
        x = pts['x'].reshape(h, w)
        y = pts['y'].reshape(h, w)
        z = pts['z'].reshape(h, w)
        rgb_u32 = pts['rgb'].copy().view(np.uint32)
        r = ((rgb_u32 >> 16) & 0xFF).reshape(h, w).astype(np.uint8)
        g = ((rgb_u32 >> 8) & 0xFF).reshape(h, w).astype(np.uint8)
        b = (rgb_u32 & 0xFF).reshape(h, w).astype(np.uint8)
        rgb_img = np.dstack([r, g, b])

        hsv = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
        mask = None
        for lo, hi in COLOR_HSV.get(color, COLOR_HSV['blue']):
            m = cv2.inRange(hsv, lo, hi)
            mask = m if mask is None else cv2.bitwise_or(mask, m)

        if debug_save:
            cv2.imwrite('/tmp/box_rgb_debug.png', cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR))
            cv2.imwrite('/tmp/box_mask_debug.png', mask)

        valid = (mask.astype(bool) & np.isfinite(x) & np.isfinite(y) & np.isfinite(z))
        if gate is not None:
            gx0, gx1, gy0, gy1, gz0, gz1 = gate
            valid = valid & (x >= gx0) & (x <= gx1) & (y >= gy0) & (y <= gy1) \
                          & (z >= gz0) & (z <= gz1)
        n_valid = int(valid.sum())
        if n_valid < MIN_VALID_PIXELS:
            log.error(f'[detect] only {n_valid} valid {color} pixels found (need '
                      f'>= {MIN_VALID_PIXELS}){" in gate" if gate else ""} '
                      f'-- box not found')
            return None

        cx, cy, cz = float(x[valid].mean()), float(y[valid].mean()), float(z[valid].mean())
        log.info(f'[detect] {n_valid} px -> centroid ({cx:.3f},{cy:.3f},{cz:.3f}) '
                  f'in {cloud.header.frame_id}')

        # NOTE: the gz-sensors RGBD point cloud generator emits xyz data in
        # the classical (non-optical) camera axis convention -- X-forward,
        # Y-left, Z-up -- even though the message's frame_id names the
        # *optical* frame. Verified empirically: interpreting the points as
        # the (non-optical) camera link frame is what lines up with the known
        # box position after transforming into base_link.
        point = PointStamped()
        point.header = cloud.header
        point.header.frame_id = cloud_frame
        point.point.x, point.point.y, point.point.z = cx, cy, cz
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', cloud_frame, rclpy.time.Time(),
                timeout=RclDuration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            log.error(f'[detect] TF lookup base_link <- {cloud_frame} failed: {e}')
            return None
        point_base = tf2_geometry_msgs.do_transform_point(point, tf)
        bx, by, bz = point_base.point.x, point_base.point.y, point_base.point.z

        if abs(bz - EXPECTED_BOX_Z) > 0.02:
            log.warn(f'[detect] detected z={bz:.3f} differs from expected ground '
                      f'box z={EXPECTED_BOX_Z:.3f} by more than 2cm')

        log.info(f'[detect] box in base_link: ({bx:.3f}, {by:.3f}, {bz:.3f})')
        return (bx, by, bz)

    # --- sequence ------------------------------------------------------------
    def _attempt_grasp(self, bx, by):
        """One pre-grasp -> descend -> close on the box at (bx, by). Returns True
        only if a box is actually pinched between the jaws afterwards (verified
        via the finger positions), so a miss is caught instead of assumed."""
        log = self.get_logger()

        # pre-grasp above the box. If planning here fails the box is beyond a
        # comfortable z-down reach; report it so the caller retries/repositions
        # rather than blindly descending from the scan pose and missing.
        if not self.move_pose(bx, by, APPROACH_Z, 0.0, label='pre-grasp'):
            log.warn('[grasp] pre-grasp unreachable -- box too far for a clean grasp')
            return False

        # descend onto the box (remove it from the scene so the jaws may
        # surround it without a false collision), then close.
        self.arm.remove_collision_object(BOX_ID)
        time.sleep(0.3)
        self.move_pose(bx, by, GRASP_Z, 0.0, cartesian=True, label='descend')
        self.gripper(GRIP_CLOSED, 'grasp')

        if not self.grasp_is_holding():
            log.warn('[grasp] jaws closed on air (no box between fingers)')
            return False
        log.info('[grasp] box held between the jaws')
        return True

    def pick_up_box(self):
        """Wrist-cam scan + grasp + lift the box, then hold it in the compact
        CARRY pose. Returns True only after VERIFYING the box is actually held
        (retries the scan+grasp up to MAX_GRASP_ATTEMPTS times, and returns
        False if it never catches the box, so the caller never proceeds as if
        it picked when it didn't). On success the box stays attached to the
        gripper so a caller can drive the base before place_box_down()."""
        log = self.get_logger()
        log.info('=== PICK UP: START ===')
        sx, sy, sz = self.scan_position

        for attempt in range(1, MAX_GRASP_ATTEMPTS + 1):
            log.info(f'--- grasp attempt {attempt}/{MAX_GRASP_ATTEMPTS} ---')

            # 0) open, move the wrist camera over the workspace, and detect the
            #    box afresh (so a box nudged by a previous miss is re-located).
            self.gripper(GRIP_OPEN, 'open')
            self.move_pose(sx, sy, sz, label='scan',
                           quat_xyzw=scan_quat(self.scan_pitch))
            time.sleep(0.5)
            detection = self.detect_box_pose()
            if detection is None:
                log.warn(f'[pick] no box detected at scan pose (attempt {attempt})')
                continue
            bx, by, _bz = detection
            box_xy = (bx, by)

            # box known to MoveIt for pre-grasp/transport awareness + RViz
            self.add_box(box_xy)

            # 1+2) pre-grasp -> descend -> close, and verify the jaws hold it
            if not self._attempt_grasp(bx, by):
                self.arm.remove_collision_object(BOX_ID)
                continue

            # 3) attach the box so MoveIt carries it (and RViz shows it grasped)
            self.add_box(box_xy, z_center=GROUND_Z + BOX_SIZE / 2.0)
            self.arm.attach_collision_object(
                id=BOX_ID, link_name=GRASP_LINK, touch_links=FINGER_LINKS)
            time.sleep(0.5)

            # 4) lift straight up (cartesian), then tuck into the compact carry
            #    pose. The carry hop is a larger reposition, so use a slow joint
            #    plan rather than cartesian; slow keeps the friction-held box
            #    from being jerked loose.
            self.move_pose(bx, by, APPROACH_Z, 0.0, cartesian=True, label='lift')
            cx, cy, cz = CARRY_POSITION
            # strict: never fall back to an unseeded pose plan while holding
            # the box (see _move_pose_direct) -- it can pick a wildly
            # different joint solution and swing the arm around.
            carry_ok = self.move_pose(cx, cy, cz, 0.0, cartesian=False,
                                      label='carry', strict=True)

            # 5) confirm the box survived the lift + carry (didn't slip out).
            if not carry_ok or not self.grasp_is_holding():
                log.warn('[pick] box slipped during lift/carry -- retrying')
                self.arm.detach_collision_object(BOX_ID)
                self.arm.remove_collision_object(BOX_ID)
                # If the carry move itself failed (rather than the box slipping
                # loose on its own), the jaws are still PHYSICALLY closed on it
                # -- open them so the retry starts from an empty gripper.
                if not carry_ok:
                    self.gripper(GRIP_OPEN, 'release-after-failed-carry')
                continue

            log.info('=== PICK UP: DONE (box held) ===')
            return True

        log.error(f'=== PICK UP: FAILED after {MAX_GRASP_ATTEMPTS} attempts '
                  f'(no box grasped) ===')
        self.arm.remove_collision_object(BOX_ID)
        return False

    def grab_below(self, grasp_z=GRASP_Z, color='blue', x_offset=FRONT_X_OFFSET):
        """Claw grab: the `color` box has been driven to the claw stop distance
        by claw_approach. Take a fresh front-camera read of it, descend straight
        onto that spot (to grasp_z -- raise it for a box on a table), close,
        verify, then lift and tuck into the carry pose. No scan/reorientation --
        the gripper stays pointing down the whole time. Returns True only if the
        box is actually held (verified via the fingers); a miss is retried by
        the caller re-centring and calling again.

        The descent target is wherever the box actually IS (bounded by
        MAX_REACH_X), not a fixed point under the ready pose -- the base only
        has to park the box within reach, not directly beneath the gripper. See
        CLAW_STOP_X in mission.py for why that distinction is what makes a table
        pick possible at all."""
        log = self.get_logger()
        log.info('=== CLAW GRAB: descend straight down ===')
        self.gripper(GRIP_OPEN, 'open')
        # Fresh read of the box now under the gripper, then descend onto it.
        # x_offset corrects the front camera's forward bias; the default is the
        # value measured for a box on the GROUND. That bias does not hold for a
        # box raised on a table, and the box is only 4.5 cm wide, so applying it
        # there puts the jaws on the box's far edge and shoves it away instead of
        # grasping -- table picks pass a smaller offset.
        det = self.detect_box_front(timeout_sec=1.5, color=color)
        if det is None:
            log.warn('[claw] box not seen for grab')
            return False
        bx = min(MAX_REACH_X, det[0] + x_offset)
        by = det[1]
        self.move_pose(bx, by, grasp_z, 0.0, cartesian=True,
                       label='claw descend', quat_xyzw=zdown_quat(0.0))
        self.gripper(GRIP_CLOSED, 'grasp')
        if not self.grasp_is_holding():
            log.warn('[claw] jaws closed on air -- lifting to retry')
            self.move_pose(bx, by, READY_Z, 0.0, cartesian=True,
                           label='claw lift-empty', quat_xyzw=zdown_quat(0.0))
            return False
        log.info('[claw] box held between the jaws')
        # The grasp is verified, so make it rigid before any lifting or driving
        # happens -- everything below this point (lift, carry pose, then a
        # multi-metre Nav2 drive to the column) is where the friction hold used
        # to lose the box.
        self.attach_box(color)

        # attach so MoveIt carries it + RViz shows it, lift straight up, carry.
        # The grasp frame is now fr3_hand_tcp, which sits BETWEEN the jaws - so
        # the held box's centre is at the grasp z itself, not 0.0575 m below it
        # as it was when poses were commanded for the gripper BODY.
        self.add_box((bx, by), z_center=grasp_z)
        self.arm.attach_collision_object(
            id=BOX_ID, link_name=GRASP_LINK, touch_links=FINGER_LINKS)
        time.sleep(0.5)
        self.move_pose(bx, by, READY_Z, 0.0, cartesian=True,
                       label='claw lift', quat_xyzw=zdown_quat(0.0))
        cx, cy, cz = CARRY_POSITION
        # strict: never fall back to an unseeded pose plan while holding the
        # box -- that fallback picks ANY IK solution, including ones that
        # swing the joints all the way around, which can shake the box loose.
        if not self.move_pose(cx, cy, cz, 0.0, cartesian=False, label='carry',
                              strict=True):
            log.warn('[claw] carry move failed -- releasing so the retry '
                     'starts from a clean, empty-gripper state')
            self.arm.detach_collision_object(BOX_ID)
            self.arm.remove_collision_object(BOX_ID)
            # The jaws are still PHYSICALLY closed on the box here (detaching
            # only clears MoveIt's bookkeeping) -- open them too, otherwise the
            # retry re-approaches with a box already clamped in the gripper,
            # which is what turned one failed carry into a fully failed pick.
            self.gripper(GRIP_OPEN, 'release-after-failed-carry')
            return False
        if not self.grasp_is_holding():
            log.warn('[claw] box slipped during lift/carry')
            self.arm.detach_collision_object(BOX_ID)
            self.arm.remove_collision_object(BOX_ID)
            # Fingers report empty, so whatever is still welded is not really
            # grasped -- drop the weld too rather than carting an invisible box.
            self.detach_box(log_label='after slip during lift/carry')
            return False
        log.info('=== CLAW GRAB: DONE (box held) ===')
        return True

    def place_box_down(self, place_xy=None):
        """Place the currently-held box down at place_xy (base_link frame,
        default PLACE_XY), release, and return the arm home."""
        log = self.get_logger()
        px, py = place_xy if place_xy is not None else PLACE_XY
        place_yaw = math.atan2(py, px)
        log.info('=== PLACE DOWN: START ===')

        # from carry: reposition over the place location (slow joint plan, as
        # for the carry hop), then lower straight down (cartesian).
        self.move_pose(px, py, APPROACH_Z, place_yaw, cartesian=False, label='to place')
        self.move_pose(px, py, GRASP_Z, place_yaw, cartesian=True, label='place-down')
        self.arm.detach_collision_object(BOX_ID)
        time.sleep(0.3)
        self.gripper(GRIP_OPEN, 'release')

        # retreat and go home
        self.move_pose(px, py, APPROACH_Z, place_yaw, cartesian=True, label='retreat')
        self.arm.remove_collision_object(BOX_ID)
        self.move_config(HOME_CONFIG, 'home')
        log.info('=== PLACE DOWN: DONE ===')

    def run(self):
        """Stationary pick-and-place: pick the box up and place it at PLACE_XY
        relative to the current base pose (unchanged external behavior)."""
        self.get_logger().info('=== PICK AND PLACE: START ===')
        if not self.pick_up_box():
            return
        self.place_box_down()
        self.get_logger().info('=== PICK AND PLACE: DONE ===')


def main():
    rclpy.init()
    node = PickAndPlace()
    ex = rclpy.executors.MultiThreadedExecutor(4)
    ex.add_node(node)
    t = threading.Thread(target=node.run, daemon=True)
    # give MoveIt/action servers a moment, then run the sequence
    time.sleep(3.0)
    t.start()
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
