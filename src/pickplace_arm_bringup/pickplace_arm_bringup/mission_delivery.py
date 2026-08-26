#!/usr/bin/env python3
"""One robot's half of the sample-transport job: collect, deliver, park.

WHAT THIS IS, AND WHAT IT IS NOT. This node knows how to run ONE errand from end
to end and nothing else. It does not decide which rack it is fetching, which
slot the rack goes in, or which parking vertex it ends on -- it is told the
first and it ASKS for the other two, at the moment they are needed. Deciding is
task_manager.py's job, and keeping the two apart is what makes "first available
position in a row" mean something: the slot is chosen when the robot arrives at
the delivery table, not when the fleet was dispatched.

THE ERRAND
    1  drive to the collection table's standoff
    2  claw-pick the rack there by colour
    3  drive to the delivery table's standoff
    4  ASK the manager for a slot            <- allocation happens here
    5  place the rack in that slot
    6  ASK the manager for a parking vertex  <- and here
    7  drive there and stop

EVERYTHING IN STEPS 1-2 IS THE WAREHOUSE MISSION'S CODE, UNCHANGED. claw_pick,
claw_approach and grab_below are inherited from Mission/PickAndPlace and are the
same routines the single-robot run flies, down to CLAW_STOP_X. The rack is the
same payload Mission2Hospital already carries, so PAYLOAD_GRIP_HEIGHT and
carry_height() apply as they stand. What is genuinely new is only step 5,
because a table with three slots in a row is not a coloured column.

WHY PLACEMENT DEAD-RECKONS INSTEAD OF SERVOING ON A COLOUR. The warehouse run
centres the base on the target using the front camera, because each column is
painted its box's colour. A delivery table is not painted anything, and it
cannot be: any rack may go in any slot, so a colour per slot would have to mean
"this slot is for red", which is exactly the fixed assignment the task manager
exists to avoid.

The alternative works because of a distinction that is easy to miss. Nav2 stops
when it believes it is within xy_goal_tolerance (0.20 m) of the goal, so ARRIVAL
error against the commanded pose was measured at 0.34-0.47 m -- larger than the
slot spacing. But the robot's own POSE ESTIMATE at that moment is good to about
0.10 m (measured 0.06, 0.03 and 0.10 m at the three collection tables after
drives of 29-40 m). So the slot's position in base_link, computed from the live
map -> base_link transform rather than from where the robot was told to stand,
carries AMCL's error and not Nav2's. That is what this does.
"""
import math
import sys
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from sensor_msgs.msg import LaserScan
from sensor_msgs_py import point_cloud2
from rclpy.duration import Duration as RclDuration
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from pickplace_arm_bringup.mission_2 import Mission2Hospital, TIGHT_YAW_TOLERANCE, \
    DEFAULT_YAW_TOLERANCE
from pickplace_arm_bringup.pick_and_place import (
    BOX_ID, BOX_SIZE, GRIP_OPEN, GROUND_Z, HOME_CONFIG, PRECISE_ACC,
    PRECISE_VEL, SCENE_SYNC_RELEASE, zdown_quat)
from pickplace_arm_bringup import rack_table_layout as RT


class DeliveryMission(Mission2Hospital):
    """Collect one rack, deliver it to a slot, park.

    Subclasses Mission2Hospital rather than Mission2 so that everything the rack
    needs -- the grasp topics pointing at rack_<colour>, PAYLOAD_GRIP_HEIGHT,
    PAYLOAD_FLOOR_Z, the colour gate that rejects a furnished hospital -- is
    already right. The layout attributes it inherits describe the SINGLE-robot
    scenario and are overwritten per errand in run_errand().
    """

    # Where the fingertips must be for the rack's tray to rest on the delivery
    # table's top, in base_link. Same arithmetic as a column placement: the top's
    # height above the floor, lifted into base_link, plus the height of the
    # grasped feature above the payload's own underside.
    DELIVERY_PLACE_Z = GROUND_Z + RT.TABLE_TOP + RT.RACK_GRIP_HEIGHT

    # THE REACH LIMIT IS A SPHERE, NOT A NUMBER, and treating it as a number is
    # what broke the placement.
    #
    # A first version capped x at 0.88, worked out for a slot straight ahead at
    # the PLACE height. Both assumptions fail in practice. The robot does not
    # arrive perfectly centred, so the slot carries a lateral offset -- measured
    # 0.22 m on one run -- and the arm's first move is not to the place height
    # but to the OVER-slot waypoint above it. Together those put the commanded
    # pose at (0.88, -0.22, 0.43), which is 0.849 m from the arm base: 99.3% of
    # the FR3's 0.855 m reach, i.e. fully extended, and MoveIt returned
    # NO_IK_SOLUTION exactly as it should.
    #
    # So the cap is computed per placement from the ACTUAL y and z, against a
    # reach budget that keeps the elbow off the singularity.
    ARM_BASE_X = 0.0799          # fr3_link0 ahead of base_link
    ARM_BASE_Z = GROUND_Z + 0.3837
    ARM_REACH = 0.855
    # 0.95, up from 0.93. Seeded IK does start failing before 1.0 -- the
    # measured NO_IK_SOLUTION above was at 0.849 m, i.e. 99.3% -- so this stays
    # a fraction and not 1.0. But 0.93 was costing 0.017 m of reach that the
    # delivery standoff now needs: the base has to stand 0.8237 m clear of the
    # table's solid collision block (see DELIVERY_NAV_STANDOFF in
    # rack_table_layout), and the arm has to cover that distance from a pose the
    # robot can actually occupy. 0.95 gives 0.812 m of usable reach against a
    # measured failure at 0.849, so it keeps 0.037 m of margin to anything that
    # has ever actually failed.
    ARM_REACH_FRACTION = 0.95

    def _max_x_for(self, py, pz):
        """Largest base_link x the arm can reach at this y and z, or None."""
        budget = (self.ARM_REACH * self.ARM_REACH_FRACTION) ** 2 \
            - py ** 2 - (pz - self.ARM_BASE_Z) ** 2
        if budget <= 0.0:
            return None
        return self.ARM_BASE_X + math.sqrt(budget)

    def __init__(self):
        super().__init__()
        self.ns = self.get_namespace().strip('/')
        self._task = None
        self._task_lock = threading.Lock()
        # THE LIDAR IS THE ONLY SENSOR ON THIS ROBOT THAT CAN SEE A WHOLE
        # DELIVERY TABLE AT PLACING RANGE, which is what _table_centre_offset
        # needs and why this subscription exists. The front camera cannot: its
        # horizontal FOV is 1.57 rad and at the placing standoff the table face
        # is only ~0.12 m in front of the lens, so it sees about +/-0.12 m of a
        # 1.327 m table -- both edges are far outside the frame. The lidar spans
        # 190 deg from base_link (0.410, 0, 0.182), so at the same standoff the
        # table's edges sit at about +/-50 deg, well inside the arc.
        #
        # THE BEAM HEIGHT IS WHAT MAKES THIS WORK AT ALL, and it is a 9.5 mm
        # margin: the laser centre is 0.3143 m above the floor and the table top
        # is 0.3238 m, so the scan passes just UNDER the lip and returns the
        # front face rather than skimming across the top. The face is a solid
        # block from the floor up (see TABLE_NEAR_FACE), so anything below the
        # top reads the same rectangle -- but do not raise the sensor without
        # revisiting this.
        self._scan_lock = threading.Lock()
        self._scan = None
        self.create_subscription(
            LaserScan, 'scan', self._scan_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))
        # Why the errand ended, set by _fail and read by run() when it hands the
        # outcome to _park. Empty means it did not fail.
        self._fail_reason = ''
        # Matched to the manager's latched publisher: a mission node that is
        # still constructing when the task goes out still receives it, and
        # receives it exactly once.
        task_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(String, 'task', self._on_task, task_qos)
        self.status_pub = self.create_publisher(String, 'mission_status', 10)
        # THE CLAIM CLIENTS LIVE ON THEIR OWN NODE, WITH THEIR OWN EXECUTOR.
        #
        # They were first created on this node, in its default callback group,
        # and the response never arrived: the manager granted r2 delivery slot 0
        # and logged it, while the mission node sat on an un-completed future
        # for the full timeout and then reported "no delivery slot available"
        # for a slot it had already been given. Moving them to a
        # ReentrantCallbackGroup did not fix it either.
        #
        # The reason is that run() is a plain thread polling fut.done() while
        # this node's MultiThreadedExecutor is busy with everything a mission
        # node subscribes to -- point clouds, joint states, TF, and pymoveit2's
        # own machinery, which does its own blocking waits inside those same
        # threads. Rather than keep guessing at callback-group interactions, the
        # claim gets a node nobody else spins, so `spin_until_future_complete`
        # on it is unambiguous and cannot be starved by the mission's work.
        self._claim_node = rclpy.create_node(
            'claim_client', namespace=self.get_namespace())
        self._slot_client = self._claim_node.create_client(Trigger, 'claim_slot')
        self._park_client = self._claim_node.create_client(Trigger, 'claim_park')
        self.get_logger().info(f'[{self.ns}] delivery mission ready, waiting for a task')

    # --- talking to the task manager -----------------------------------------
    def _on_task(self, msg):
        with self._task_lock:
            self._task = msg.data

    def take_task(self):
        with self._task_lock:
            t, self._task = self._task, None
        return t

    def say(self, state, detail=''):
        """Report progress. The manager watches these to know when an errand is
        done; RViz shows them as the fleet's status board."""
        self.status_pub.publish(String(data=f'{self.ns}|{state}|{detail}'))
        self.get_logger().info(f'[{self.ns}] {state} {detail}')

    def _fail(self, reason):
        """Record why the errand ended and return False, WITHOUT announcing it.

        The announcement is deferred to _park, which publishes the outcome and
        then drives the robot off the floor. Saying 'failed' here instead would
        be the terminal status, and the robot would still have to cross the
        lobby afterwards -- so the reason is stored and _park says it at the
        moment the manager should act on it.
        """
        self._fail_reason = reason
        self.get_logger().error(f'[{self.ns}] errand failed: {reason}')
        return False

    def _claim(self, client, what, wait_sec=0.0):
        """Ask the manager for a slot or a parking vertex. Returns an int index,
        or None. This is a REQUEST rather than a lookup on purpose: the answer
        depends on what the other robots have already taken.

        `wait_sec` > 0 keeps asking while the manager says the resource is BUSY
        rather than giving up, which is what turns the delivery lock from a
        check into a queue -- see run_errand for why the slot is claimed before
        the drive rather than on arrival.
        """
        if not client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error(f'[{self.ns}] no {what} service -- is the '
                                    f'task manager running?')
            return None
        deadline = time.time() + max(wait_sec, 0.0)
        announced = False
        last_said = 0.0
        while True:
            fut = client.call_async(Trigger.Request())
            # Safe precisely because _claim_node is spun by nobody else.
            rclpy.spin_until_future_complete(self._claim_node, fut,
                                             timeout_sec=30.0)
            if not fut.done() or fut.result() is None:
                self.get_logger().error(f'[{self.ns}] {what} request timed out')
                return None
            res = fut.result()
            if res.success:
                return int(res.message)
            # "in use by <ns>" is a queue, not a refusal. Anything else -- no
            # free slot at all, say -- is final.
            if 'in use' in res.message and time.time() < deadline:
                # SAY SO PERIODICALLY, NOT ONCE. This loop polls every 2 s for
                # up to wait_sec, and it used to announce itself a single time
                # and then go silent -- so a robot that had picked its rack,
                # parked on its queue spot and was simply waiting its turn was
                # indistinguishable, from the terminal, from one that had hung.
                # That is what "it got stuck and didn't move anymore after
                # picking" looks like from the outside, and the robot was fine.
                now = time.time()
                if not announced or now - last_said >= 15.0:
                    waited = now - (deadline - max(wait_sec, 0.0))
                    self.get_logger().info(
                        f'[{self.ns}] queued for the delivery table, waiting '
                        f'{waited:.0f}s ({res.message})')
                    announced = True
                    last_said = now
                time.sleep(2.0)
                continue
            self.get_logger().error(f'[{self.ns}] {what} refused: {res.message}')
            return None

    # --- placing on the delivery table ---------------------------------------
    def _slot_in_base_link(self, slot_xy):
        """Where a delivery slot is, in the robot's own frame, RIGHT NOW.

        Computed from the live map -> base_link transform rather than from the
        pose Nav2 was asked to reach, which is the whole point -- see the module
        docstring for the 0.10 m against 0.47 m difference that makes.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_frame('base_link'), 'map', rclpy.time.Time(),
                timeout=RclDuration(seconds=2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'[{self.ns}] base_link <- map failed: {e}')
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * slot_xy[0] - s * slot_xy[1] + t.x,
                s * slot_xy[0] + c * slot_xy[1] + t.y)

    # The band of the front camera's cloud that is the delivery table's FRONT
    # FACE and nothing else, in base_link. Heights are above the floor:
    # the face runs 0 .. TABLE_TOP (0.324), the racks already standing on the
    # table start ABOVE 0.324, and the floor is below 0.05. 0.08..0.26 sits
    # inside the face with margin at both ends.
    FACE_Z_LO = GROUND_Z + 0.08
    FACE_Z_HI = GROUND_Z + 0.26
    FACE_HALF_WIDTH = 0.20      # m either side of the bow
    FACE_X_MIN, FACE_X_MAX = 0.25, 1.30

    def _table_face_ahead(self, timeout_sec=2.0):
        """Distance from base_link to the delivery table's front face, MEASURED.

        WHY THIS EXISTS. Everything else about the placement is computed from
        map -> base_link, so the rack lands wherever AMCL thinks the slot is,
        and AMCL's ABSOLUTE error goes straight into the result. Measured on a
        run where every check passed and the slot readings looked healthy
        (+0.869 and +0.833, both comfortably in reach):

            rack_red   ended 0.19 m short of its slot, 0.045 m from the edge
            rack_green ended past the edge, on the floor at z = 0.063

        No amount of geometry fixes that: the margin between a rack and the
        table's near edge is D - 0.4937 - clearance, which this robot and this
        arm cap at about 0.19 m, and the error to absorb is the same size.

        The camera does not have that problem. front_camera_link is bolted to
        the chassis at base_link (0.425, 0, 0.091) with zero rpy, and the
        table's front face is a flat vertical wall 0.55 m in front of it that
        spans the whole band between the floor and the table top. Measuring it
        gives the ONE distance the placement actually needs -- how far the table
        is -- with no reference to where AMCL thinks anybody is.

        Returns the base_link x of the face, or None if it cannot be measured.
        """
        log = self.get_logger()
        frame = self.tf_frame('front_camera_link')
        with self._front_lock:
            self._front_cloud = None
        deadline = time.time() + timeout_sec
        cloud = None
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            with self._front_lock:
                cloud = self._front_cloud
            if cloud is not None:
                break
        if cloud is None:
            log.warn(f'[{self.ns}] no front cloud to measure the table with')
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_frame('base_link'), frame, rclpy.time.Time(),
                timeout=RclDuration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            log.warn(f'[{self.ns}] TF for the table measurement failed: {e}')
            return None
        pts = point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'),
                                       skip_nans=False)
        # The cloud is in the camera BODY convention (X forward, Y left, Z up)
        # and the camera carries no rotation relative to base_link, so this is
        # a pure translation -- see the note in PickAndPlace._detect for how
        # that convention was established.
        t = tf.transform.translation
        bx = np.asarray(pts['x'], dtype=float).ravel() + t.x
        by = np.asarray(pts['y'], dtype=float).ravel() + t.y
        bz = np.asarray(pts['z'], dtype=float).ravel() + t.z
        keep = (np.isfinite(bx) & np.isfinite(by) & np.isfinite(bz)
                & (np.abs(by) <= self.FACE_HALF_WIDTH)
                & (bz >= self.FACE_Z_LO) & (bz <= self.FACE_Z_HI)
                & (bx >= self.FACE_X_MIN) & (bx <= self.FACE_X_MAX))
        n = int(keep.sum())
        if n < 200:
            log.warn(f'[{self.ns}] only {n} points on the table face -- '
                     f'not measuring it')
            return None
        # The 10th percentile, not the minimum: the face is flat, so the bulk of
        # these points share one x, and a percentile is immune to the odd stray
        # return that a hard min would latch onto.
        face = float(np.percentile(bx[keep], 10.0))
        log.info(f'[{self.ns}] delivery table face measured at base_link '
                 f'x={face:.3f} ({n} points)')
        return face

    # Where the table's face should read from base_link when the robot is
    # standing at the delivery standoff. Derived from the same three numbers the
    # standoff is, so it cannot drift away from them.
    FACE_TARGET = (RT.DELIVERY_NAV_STANDOFF - RT.DELIVERY_SLOT_LOCAL_Y
                   - RT.TABLE_NEAR_FACE)
    FACE_TOL = 0.04           # m; inside this, do not bother moving
    FACE_STEP_MAX = 0.45      # m; a bigger correction than this is a bad reading
    FACE_MIN_SAFE = 0.52      # m; never drive so the face is nearer than this

    # Deliberately slow: at 0.10 m/s the base reaches speed in 0.04 s under the
    # 2.5 m/s^2 limit, so "time x speed" is the distance to within a couple of
    # percent, and the caller re-measures anyway.
    CREEP_SPEED = 0.10

    def _creep_blind(self, dist):
        """Drive `dist` metres forward (negative = back), open loop.

        _drive_blind takes a VELOCITY and a DURATION, not a distance -- the
        existing call sites pass (-0.35, 4.0) and mean 1.4 m of reverse, which
        is easy to misread as 0.35 m. This wrapper takes the distance, because
        every use here is a correction the camera just measured in metres.
        """
        if abs(dist) < 0.01:
            return
        v = self.CREEP_SPEED if dist > 0 else -self.CREEP_SPEED
        self._drive_blind(v, abs(dist) / self.CREEP_SPEED)
        self._stop_base()

    def _scan_cb(self, msg):
        with self._scan_lock:
            self._scan = msg

    # How wide the table may MEASURE and still be believed. The top is 1.327 m
    # (TABLE_LONG). A reading far under that means an edge was occluded -- by a
    # rack already standing on the table, by another robot -- and the midpoint
    # of a partial face is not the middle of the table. A reading far over it
    # means something else in the lobby got into the band. Either way the
    # correction is refused rather than guessed, and the placement falls back to
    # the map estimate that was in use before.
    TABLE_WIDTH_LO, TABLE_WIDTH_HI = 1.15, 1.50
    # How far off the FITTED face line a return may sit and still count as part
    # of it. The fit has already taken the yaw out, so this absorbs beam noise
    # and the face's own roughness -- nothing geometric.
    FACE_PLANE_TOL = 0.05
    # How far either side of the camera's face reading to LOOK for the face
    # before fitting it. This one does have to cover the yaw: at 15 degrees the
    # table's far edge is 0.172 m deeper than its near edge, so 0.30 holds the
    # whole face with room to spare, and the line fit then takes the tilt out.
    # The lobby wall behind the table is metres away and never enters it.
    FACE_SEARCH_TOL = 0.30

    def _table_centre_offset(self, face_x):
        """Lateral offset from the bow to the delivery table's CENTRE, measured.

        WHY THIS EXISTS, WITH THE NUMBERS THAT PUT IT HERE. Depth was taken off
        AMCL by _table_face_ahead; this does the same for the sideways axis, and
        until it did, two racks landed on top of each other. Measured from
        Gazebo ground truth on the 17:04 run:

            rack_red    slot 0 (-3.240,+10.065)   ended (-3.336,+10.025)  0.104 m off
            rack_green  slot 1 (-3.500,+10.065)   ended (-3.391,+10.121)  0.122 m off

        NEITHER ROBOT WAS BADLY LOCALISED. 0.104 and 0.122 m are both inside the
        0.15 m this code calls AMCL's ordinary error. But the two errors pointed
        AT EACH OTHER and closed 0.205 m of the 0.260 m between the slots,
        leaving 0.111 m -- less than the 0.160 m a rack is wide. Green came to
        rest 0.0163 m above the table top at 8.2 degrees of tilt, i.e. balanced
        on red's edge.

        So the quantity that matters is not one robot's error but the
        DIFFERENCE between two, and no per-robot tolerance controls that. What
        removes it is giving every robot the same physical reference instead of
        three independent estimates of it: the table itself.

        Returns the table centre's y in base_link, or None if the face could not
        be measured well enough to trust -- in which case the caller keeps the
        map estimate rather than acting on a bad correction.
        """
        log = self.get_logger()
        with self._scan_lock:
            scan = self._scan
        if scan is None:
            log.warn(f'[{self.ns}] no scan to find the table centre with')
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_frame('base_link'), scan.header.frame_id,
                rclpy.time.Time(), timeout=RclDuration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            log.warn(f'[{self.ns}] TF for the table centre failed: {e}')
            return None
        # THROUGH TF, NOT BY HAND. The lidar is mounted inverted (roll = pi, see
        # the URDF), so a beam's bearing in lidar_link is NOT its bearing in
        # base_link -- the sign of y flips. Transforming the points is what
        # AMCL and the costmaps do with this same scan, and doing anything
        # cleverer here is how the correction would come out mirrored.
        # THE FULL ROTATION MATRIX, NOT A YAW. Extracting a yaw from this
        # particular quaternion and rotating in the plane would be wrong twice
        # over: the mount carries a roll of pi, and the usual yaw formula
        # assumes there is no roll to begin with. Building R from the quaternion
        # costs three lines and is correct whatever the mount turns into.
        t, q = tf.transform.translation, tf.transform.rotation
        xx, yy, zz, ww = q.x, q.y, q.z, q.w
        r00 = 1.0 - 2.0 * (yy * yy + zz * zz)
        r01 = 2.0 * (xx * yy - zz * ww)
        r10 = 2.0 * (xx * yy + zz * ww)
        r11 = 1.0 - 2.0 * (xx * xx + zz * zz)
        rng = np.asarray(scan.ranges, dtype=float)
        ang = scan.angle_min + np.arange(rng.size) * scan.angle_increment
        good = np.isfinite(rng) & (rng >= scan.range_min) & (rng <= scan.range_max)
        # Zero the dead beams before the trig rather than after: a scan carries
        # inf for "nothing out there", and inf * cos is a nan that then makes
        # every comparison below a warning as well as a False.
        rng = np.where(good, rng, 0.0)
        # The scan is planar in its own frame, so the beam's z is 0 and only the
        # top-left 2x2 of R contributes to base_link x and y.
        lx = rng * np.cos(ang)
        ly = rng * np.sin(ang)
        bx = r00 * lx + r01 * ly + t.x
        by = r10 * lx + r11 * ly + t.y
        # FIT THE FACE, DO NOT ASSUME IT IS SQUARE TO THE BOW. A window at a
        # constant x measures the table's width only if the robot is perfectly
        # square to it, and it never is: the table's far edge is 0.664 m off the
        # bow, so 13 degrees of yaw puts that edge 0.149 m deeper than the near
        # one and a +/-0.12 window clips it off. Measured, r2 on the 17:22 run:
        # "table face measured 1.061 m wide (205 returns), not the expected
        # 1.327 -- refusing the lateral correction". The face was there and 205
        # beams were on it; 13.1 degrees of yaw was all it took to lose 0.266 m
        # of it and with it the whole correction, which is why r2's rack landed
        # 0.109 m out while r3's -- corrected -- landed 0.050 m out.
        #
        # So: take a window generous enough to hold the whole face at any yaw
        # this robot can arrive with, fit a line to it, and measure the width
        # ALONG THAT LINE. The tolerance then has only beam noise to absorb.
        near = (good
                & (np.abs(bx - face_x) <= self.FACE_SEARCH_TOL)
                & (np.abs(by) <= RT.TABLE_LONG))
        if int(near.sum()) < 25:
            log.warn(f'[{self.ns}] only {int(near.sum())} lidar returns near '
                     f'the table face at x={face_x:.3f} -- not measuring the '
                     f'centre')
            return None
        # Total least squares through the face returns: the principal axis of
        # the points IS the face, and unlike a y-on-x fit it does not blow up as
        # the face turns towards edge-on.
        fx, fy = bx[near], by[near]
        mx, my = float(fx.mean()), float(fy.mean())
        u, _sv, _vt = np.linalg.svd(np.stack([fx - mx, fy - my]))
        dirx, diry = float(u[0, 0]), float(u[1, 0])
        # Perpendicular distance to the fitted line, not to a constant x.
        perp = np.abs(-diry * (bx - mx) + dirx * (by - my))
        on_face = good & near & (perp <= self.FACE_PLANE_TOL)
        n = int(on_face.sum())
        # A 190 deg / 270 beam scan puts well over a hundred returns on a table
        # this close, so 25 is a floor for "saw a table", not a target.
        if n < 25:
            log.warn(f'[{self.ns}] only {n} lidar returns on the table face at '
                     f'x={face_x:.3f} -- not measuring the centre')
            return None
        # WIDTH ALONG THE FACE, not along base_link y. With the yaw taken out by
        # the fit these differ by cos(yaw) -- 2.6% at 13 degrees -- and that is
        # the difference between a reading the width check accepts and one it
        # throws away.
        s = dirx * (bx[on_face] - mx) + diry * (by[on_face] - my)
        lo, hi = float(s.min()), float(s.max())
        width = hi - lo
        if not (self.TABLE_WIDTH_LO <= width <= self.TABLE_WIDTH_HI):
            log.warn(f'[{self.ns}] table face measured {width:.3f} m wide '
                     f'({n} returns), not the expected {RT.TABLE_LONG:.3f} -- '
                     f'refusing the lateral correction')
            return None
        # The midpoint of the face, taken back into base_link. Only its y is
        # wanted: depth stays the camera's job (see _table_face_ahead), and the
        # two measurements are deliberately kept on separate sensors so a bad
        # reading on one cannot quietly move the other.
        mid = (lo + hi) / 2.0
        centre = my + diry * mid
        yaw_deg = math.degrees(math.atan2(dirx, abs(diry) if diry else 1e-9))
        log.info(f'[{self.ns}] table centre measured at base_link y={centre:+.3f} '
                 f'(face {width:.3f} m wide, {n} returns, {yaw_deg:+.1f} deg off '
                 f'square)')
        return centre

    def _square_up_on_table(self, tries=3):
        """Drive the base until the delivery table's face is at FACE_TARGET.

        This is the one part of the placement that does not go through AMCL.
        The face is a flat wall half a metre in front of the camera and the
        robot's own reach envelope is centred on its own base, so putting the
        base at a measured distance from the table is the only thing that makes
        the slot reachable and repeatable at once.

        The moves are open loop and short on purpose: each one is a correction
        the camera just measured, bounded to FACE_STEP_MAX, and re-measured
        afterwards. FACE_MIN_SAFE is the guard that keeps the bumper out of a
        table whose collision volume is a solid block (see TABLE_NEAR_FACE).
        Returns the final measured face distance, or None if it never got a
        measurement at all.
        """
        log = self.get_logger()
        face = None
        for attempt in range(1, tries + 1):
            face = self._table_face_ahead()
            if face is None:
                return None
            err = face - self.FACE_TARGET
            if abs(err) <= self.FACE_TOL:
                log.info(f'[{self.ns}] squared up on the table: face at '
                         f'{face:.3f} m (target {self.FACE_TARGET:.3f})')
                return face
            step = max(-self.FACE_STEP_MAX, min(self.FACE_STEP_MAX, err))
            # Never let a correction take the bumper into the table.
            if face - step < self.FACE_MIN_SAFE:
                step = face - self.FACE_MIN_SAFE
            if abs(step) <= 0.01:
                return face
            log.info(f'[{self.ns}] table face at {face:.3f} m, want '
                     f'{self.FACE_TARGET:.3f} -- moving {step:+.3f} m '
                     f'({attempt}/{tries})')
            self._creep_blind(step)
            time.sleep(0.6)          # let the base settle before re-measuring
        face = self._table_face_ahead()
        if face is not None:
            log.warn(f'[{self.ns}] table face settled at {face:.3f} m against a '
                     f'target of {self.FACE_TARGET:.3f} -- placing from there')
        return face

    def _settled_slot(self, slot_xy, timeout_sec=25.0):
        """The slot's position in base_link, once the pose estimate has settled.

        READING IT THE INSTANT NAV2 SAYS "ARRIVED" IS TOO EARLY, and by a lot.
        AMCL only resamples after update_min_d (0.25 m) or update_min_a
        (0.2 rad) of motion, so right after a 40 m drive that ends in an
        in-place rotation its estimate can still be catching up -- and the
        transform is what this whole placement is computed from. Measured on a
        run that failed here: 3 ms after "Nav2 reached goal" the slot came out
        at base_link (0.336, 0.818), i.e. 0.88 m away and 68 degrees off the
        bow, when ground truth put it at (1.287, -0.022). The arm was then asked
        to reach a point that was not there and MoveIt correctly returned
        NO_IK_SOLUTION. Half a minute later the same transform read (1.163,
        0.079) -- within AMCL's ordinary 0.15 m -- because it had converged.

        So: let the base settle, then sample until consecutive readings agree.
        Agreement is the condition, not a fixed sleep, because how long
        convergence takes depends on the drive that preceded it.
        """
        # THE TWO WAITS ARE NOT THE SAME WAIT, which is why only one of them
        # shrank much, and why the ACCURACY of this is unchanged.
        #
        # The first is mechanical: the base has just stopped and is rocking on
        # its tyres, and 1.0 s is past the settling time of a 46 kg chassis.
        # The second is only how often the CONVERGENCE test is sampled, and the
        # transform it reads is continuous, so sampling twice a second tests
        # exactly the same thing as once a second and reaches the answer in half
        # the wall clock. The 0.03 m agreement threshold -- the part that
        # actually decides when the estimate has settled, and therefore the part
        # that determines where the rack is put down -- is untouched.
        #
        # AMCL now resamples on 0.10 m of motion rather than 0.20
        # (amcl_hospital.yaml), so the estimate this is waiting on converges
        # sooner than it used to as well.
        log = self.get_logger()
        time.sleep(1.0)
        deadline = time.time() + timeout_sec
        prev = None
        while time.time() < deadline:
            cur = self._slot_in_base_link(slot_xy)
            if cur is None:
                time.sleep(0.3)
                continue
            if prev is not None and math.hypot(cur[0] - prev[0],
                                               cur[1] - prev[1]) < 0.03:
                return cur
            prev = cur
            time.sleep(0.5)
        if prev is not None:
            log.warn(f'[{self.ns}] slot reading never settled -- using the last '
                     f'({prev[0]:+.3f},{prev[1]:+.3f})')
        return prev

    # The slot reading that place_in_slot is willing to act on at all. Anything
    # outside this is not a reach problem, it is a POSE problem -- see
    # _slot_reading_is_sane.
    SLOT_MIN_X = 0.30            # m ahead; below this the robot is not in front
    SLOT_MAX_ABS_Y = 0.55        # m abeam; the three slots span only +/-0.30

    def _slot_reading_is_sane(self, target):
        """Is this slot reading consistent with STANDING AT THE STANDOFF?

        WHY THIS EXISTS, AND WHAT IT COST NOT TO HAVE IT. place_in_slot used to
        act on whatever _settled_slot returned. On a measured run r1 read the
        slot at base_link (-0.018, -1.7) -- 18 mm BEHIND the bumper and 1.7 m
        out to the side -- which is not a slot that is slightly too far away,
        it is a robot that is not parked in front of the table at all. Working
        back from the goal it then published, it was standing 1.4 m from the
        standoff on a heading 164 degrees off. The old code read that as "out of
        reach" and handed it to _close_in_on, which drove it FURTHER away (see
        there), and r1 held the delivery table for the rest of the run with the
        rack still in its jaws. r2 and r3 queued behind it and never placed
        either, so one bad reading cost all three deliveries.

        The standoff puts the slots at x=0.75, |y|<=0.30. A reading outside this
        window cannot be fixed by reaching or by creeping; the base has to go
        back to the standoff first.
        """
        px, py = target
        return px >= self.SLOT_MIN_X and abs(py) <= self.SLOT_MAX_ABS_Y

    def _close_in_on(self, slot_xy, px):
        """Nudge FORWARD so the slot lands inside the arm's reach, using Nav2.

        The obvious tool is _creep_forward, which measures on odom and is what
        the column placement uses. It does not work here: measured twice, it
        published cmd_vel for its full 25 s timeout and the base moved 0.000 m,
        while an external publisher on the identical topic moved the same robot
        0.71 m in six seconds. The publisher is matched (ros2 topic info -v
        lists this node on that topic, QoS compatible) so the cause is still
        open -- but the fix does not have to wait for the explanation.

        Nav2 demonstrably drives this robot, and it has the costmap, so it will
        not push the chassis into the table the way an open-loop creep could.

        THE ADVANCE IS CLAMPED POSITIVE, AND THAT IS THE WHOLE BUG THIS CARRIES
        A SCAR FROM. It used to be `advance = px - 0.78` with no floor, so any
        reading closer than 0.78 m produced a NEGATIVE advance and a goal behind
        the robot -- and because the caller only reaches here when the reading
        is already suspect, the negative case was not an edge case, it was the
        common one. Observed: "slot is -0.018 m ahead -- moving up -0.80 m",
        i.e. the recovery for "too far from the table" reversed away from it.
        Callers now screen the reading with _slot_reading_is_sane first, and
        this clamp is the second line of defence rather than the only one.
        """
        log = self.get_logger()
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.tf_frame('base_link'), rclpy.time.Time(),
                timeout=RclDuration(seconds=2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            log.error(f'[{self.ns}] map <- base_link failed: {e}')
            return False
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # Aim to leave the slot comfortably inside the cap rather than exactly
        # at it, so AMCL's own error does not put it back outside. Never
        # negative, and never more than half a metre in one step: the table is
        # 0.416 m in front of the bumper at the standoff, so a large "advance"
        # is a bad reading, not a long way to go.
        advance = min(0.5, max(0.0, px - 0.78))
        if advance <= 0.02:
            log.info(f'[{self.ns}] slot reads {px:.3f} m ahead, already inside '
                     f'the working distance -- not moving up')
            return True
        gx = t.x + advance * math.cos(yaw)
        gy = t.y + advance * math.sin(yaw)
        log.info(f'[{self.ns}] slot is {px:.3f} m ahead -- moving up '
                 f'{advance:.2f} m to ({gx:+.2f},{gy:+.2f})')
        return self.navigate_to(self.make_map_goal(gx, gy, yaw))

    def _range_to(self, xy):
        """How far this robot's base_link is from a map point, right now."""
        here = self._base_xy_in_map()
        if here is None:
            return None
        return math.hypot(here[0] - xy[0], here[1] - xy[1])

    # How far from the delivery standoff still counts as "arrived". Nav2's own
    # xy_goal_tolerance is 0.35 and AMCL carries ~0.15 on top, so anything
    # inside 0.60 m is an ordinary arrival; beyond that the robot is somewhere
    # else entirely and "Nav2 reached goal" was not true.
    STANDOFF_ARRIVED_TOL = 0.60

    def _drive_to_delivery_standoff(self, attempts=3):
        """Get to the delivery standoff, and VERIFY it rather than believe Nav2.

        NAV2 REPORTS SUCCESS FOR THIS GOAL WITHOUT MOVING THE ROBOT, and until
        that was measured every recovery built on top of it was useless.
        Observed on a full run, r2 carrying the green rack:

            [nav] sending Nav2 goal (-3.50,10.75)
            [nav] Nav2 reached goal                       <- 14.4 s later
            [r2] slot 1 reads base_link (+2.907,-0.949)   <- 3.06 m away
            [nav] sending Nav2 goal (-3.50,10.75)
            [nav] Nav2 reached goal                       <- 0.07 s later

        Seventy milliseconds. The robot never left the holding ring, 3 m back,
        and the same reading came out three times to the millimetre, so the
        transform was steady and it really was not there. The goal checker works
        in the LOCAL costmap's frame, which is odom, against a goal transformed
        out of map; the slot reading works in map. When those two disagree the
        action can succeed while the robot has not moved -- and re-sending the
        goal is then answered instantly, forever.

        So this checks the one thing that matters, the distance in the MAP
        frame, and when it does not agree with Nav2 it makes the robot MOVE.
        _relocalize rotates in place, which both re-converges AMCL and denies
        the controller the "already there" answer it kept giving.
        """
        log = self.get_logger()
        stand = RT.delivery_standoff()
        for attempt in range(1, attempts + 1):
            ok = self._navigate_with_recovery(stand, 'the delivery table',
                                              attempts=2, tight_yaw=True)
            d = self._range_to(stand[:2])
            if ok and d is not None and d <= self.STANDOFF_ARRIVED_TOL:
                return True
            if ok and d is not None:
                log.warn(f'[{self.ns}] Nav2 reported the delivery standoff '
                         f'reached, but map -> base_link puts the robot '
                         f'{d:.2f} m away (tolerance '
                         f'{self.STANDOFF_ARRIVED_TOL:.2f}) -- not believing it')
            if attempt < attempts:
                # Rotating in place is the only lever that both re-converges
                # AMCL and guarantees the next goal is not answered from a
                # standstill.
                self._relocalize('delivery standoff not actually reached')
        log.error(f'[{self.ns}] could not reach the delivery standoff')
        return False

    def _repark_at_standoff(self, why):
        """Go back to the delivery standoff and face the table.

        This is the recovery for a slot reading that says the robot is not where
        it should be. Driving to a KNOWN MAP POSE is the only thing that can fix
        that -- extrapolating from the bad reading, which is what _close_in_on
        used to be asked to do, moves the robot by an amount derived from the
        very estimate that is wrong. It goes through
        _drive_to_delivery_standoff rather than straight to Nav2, because a
        plain re-send of this particular goal is the thing that was measured to
        return success in 70 ms without moving.
        """
        self.get_logger().warn(
            f'[{self.ns}] {why} -- re-parking at the delivery standoff')
        return self._drive_to_delivery_standoff(attempts=2)

    def place_in_slot(self, slot_index, slot_xy):
        """Put the carried rack down in `slot_xy` (map frame) and let go."""
        log = self.get_logger()
        target = None
        # TWO DIFFERENT THINGS CAN BE WRONG WITH A SLOT READING, AND THEY NEED
        # OPPOSITE RESPONSES. The loop used to treat both as "out of reach".
        #
        #   THE SLOT IS AHEAD BUT TOO FAR      the robot parked a little short.
        #                                      Nudge forward: _close_in_on.
        #   THE SLOT IS NOT AHEAD AT ALL       the robot is not at the standoff,
        #                                      or not facing the table. Nothing
        #                                      about reaching or creeping fixes
        #                                      that; go back to the standoff.
        #
        # Telling them apart is what _slot_reading_is_sane does, and getting it
        # wrong is what deadlocked a whole run -- see its docstring.
        for attempt in range(3):
            target = self._settled_slot(slot_xy)
            if target is None:
                return False
            if not self._slot_reading_is_sane(target):
                log.warn(f'[{self.ns}] slot {slot_index} reads base_link '
                         f'({target[0]:+.3f},{target[1]:+.3f}), which is not a '
                         f'slot in front of the table -- treating this as a '
                         f'POSE problem, not a reach problem')
                if attempt == 2 or not self._repark_at_standoff(
                        f'slot {slot_index} reading is off the table'):
                    # ASK THE SENSORS BEFORE GIVING UP, because this branch is
                    # a statement about AMCL and not about the robot. Measured,
                    # r1 on the 17:22 run: the standoff check passed, then the
                    # slot read (+1.004,-0.903), then (+0.745,-1.216) TWICE to
                    # the millimetre -- the same reading from a robot that had
                    # been told to drive and had not moved, which is the "Nav2
                    # answers from a standstill" failure in
                    # _drive_to_delivery_standoff's docstring. Three reparks
                    # later r1 carried its rack to the car park.
                    #
                    # The camera and the lidar do not care where AMCL thinks
                    # anybody is. If one can see the table face at placing range
                    # and the other can see a full 1.327 m of it, the robot IS
                    # at the table and the placement is computable without a map
                    # pose at all -- which is the same arithmetic the healthy
                    # path below uses, just without AMCL's permission first.
                    sensed = self._slot_from_sensors_only(slot_index)
                    if sensed is not None:
                        log.warn(f'[{self.ns}] slot {slot_index} recovered from '
                                 f'the camera and lidar alone at base_link '
                                 f'({sensed[0]:+.3f},{sensed[1]:+.3f}) -- '
                                 f'placing on the sensors, not the map')
                        return self._lower_into_slot(slot_index, *sensed)
                    log.error(f'[{self.ns}] could not recover a usable slot '
                              f'reading for slot {slot_index}')
                    return False
                continue
            # Checked against the OVER-slot waypoint, which is the higher and
            # therefore tighter of the two poses the arm has to make.
            over_z = self.DELIVERY_PLACE_Z + BOX_SIZE / 2.0 + 0.04
            cap = self._max_x_for(target[1], over_z)
            if cap is not None and target[0] <= cap:
                break
            # OUT OF REACH IS RECOVERABLE, and worth recovering rather than
            # clamping. Nav2 stops within its own 0.35 m tolerance and AMCL
            # carries another 0.15, so a robot can legitimately arrive half a
            # metre short of where it meant to stand -- measured, one read the
            # slot at 1.306 m when the standoff should have put it at 0.85.
            # Clamping there would set the rack down 0.43 m short of the table.
            if attempt == 2:
                log.warn(f'[{self.ns}] still {target[0]:.3f} m out after '
                         f'{attempt + 1} approaches (reachable to '
                         f'{cap if cap is None else round(cap, 3)}) -- '
                         f'placing at the cap')
                break
            if not self._close_in_on(slot_xy, target[0]):
                log.warn(f'[{self.ns}] could not move up to the table')
                break
        px, py = target

        # SQUARE UP ON THE TABLE ITSELF, NOT ON AMCL, AND MOVE THE BASE TO DO
        # IT -- correcting the arm's target instead is what the first version of
        # this did and it does not work.
        #
        # The arm's reach is a sphere centred on the shoulder, so a slot that is
        # 0.2 m further away than the base expected is not a longer reach, it is
        # OUT of reach: the clamp then bites and the rack goes down short, near
        # the front edge, which is the failure this is here to remove. Measured,
        # first run with the face measurement in: the table read 0.783 m when
        # the standoff should have put it at 0.545, the corrected slot came out
        # at 1.048, and the arm reaches 0.844. Reaching was never going to work;
        # the base was 0.24 m too far back.
        #
        # So this drives the base until the MEASURED face is where the standoff
        # says it should be, and only then computes the slot from it. py -- which
        # slot, left to right -- still comes from the map, because the three
        # slots are a convention rather than anything the sensor can tell apart.
        # Depth is the axis a rack falls off, and depth is now measured.
        face = self._square_up_on_table()
        if face is not None:
            px = face + (RT.DELIVERY_SLOT_LOCAL_Y + RT.TABLE_NEAR_FACE)
            log.info(f'[{self.ns}] slot depth from the measured table face: '
                     f'{px:.3f} m')
            # AND THE SIDEWAYS AXIS OFF AMCL TOO, for the reason spelled out in
            # _table_centre_offset: one robot's 0.10 m error is harmless, two
            # robots' errors pointing at each other are not, and only a shared
            # physical reference removes that. Slot local x runs the opposite
            # way to base_link y here -- the robot faces the table -- which is
            # the minus. Measured centred, slot 0 sits at y = +0.260.
            #
            # MEASURED AFTER THE SQUARE-UP, NOT BEFORE. py used to be carried
            # over from _settled_slot, which reads before the base has closed in
            # on the table; the three nudges that follow move the robot up to
            # 0.43 m and any yaw they pick up walks the old lateral reading off.
            # This reads the table where the arm is actually going to work.
            centre = self._table_centre_offset(face)
            if centre is not None:
                was = py
                py = centre - RT.DELIVERY_SLOT_LOCAL_X[slot_index]
                log.info(f'[{self.ns}] slot {slot_index} lateral from the '
                         f'measured table: {py:+.3f} m (map said {was:+.3f}, '
                         f'correction {py - was:+.3f})')
        else:
            log.warn(f'[{self.ns}] could not measure the table face -- placing '
                     f'on the map estimate alone, which is what put a rack on '
                     f'the floor')
        target = (px, py)

        if not self._slot_reading_is_sane(target):
            # Belt and braces: the loop above can fall out of `break` paths
            # (could-not-move-up, or the third attempt) still holding a reading
            # it never validated. Reaching for a slot that is not in front of
            # the robot is how the arm ends up commanded into its own chassis.
            log.error(f'[{self.ns}] refusing to place: slot {slot_index} reads '
                      f'({px:+.3f},{py:+.3f}) in base_link, which is not in '
                      f'front of the table')
            return False
        log.info(f'[{self.ns}] slot {slot_index} is at base_link '
                 f'({px:+.3f},{py:+.3f})')
        return self._lower_into_slot(slot_index, px, py)

    def _slot_from_sensors_only(self, slot_index):
        """Where slot `slot_index` is, from the camera and lidar and nothing else.

        The recovery for an AMCL estimate that has diverged far enough that the
        map-frame slot reading is nonsense. Both measurements are of the TABLE,
        so neither depends on the robot knowing where it is:

            depth    the camera's face reading, squared up to FACE_TARGET
            lateral  the midpoint of the lidar's fitted face line

        Returns (px, py) in base_link, or None if either sensor could not see
        enough of the table to be believed -- in which case the caller is right
        to give up, because at that point nothing on this robot can find it.
        """
        log = self.get_logger()
        face = self._square_up_on_table()
        if face is None:
            log.warn(f'[{self.ns}] no camera view of the table face either -- '
                     f'cannot place on the sensors')
            return None
        centre = self._table_centre_offset(face)
        if centre is None:
            log.warn(f'[{self.ns}] lidar cannot see a whole table face -- '
                     f'cannot place on the sensors')
            return None
        px = face + (RT.DELIVERY_SLOT_LOCAL_Y + RT.TABLE_NEAR_FACE)
        py = centre - RT.DELIVERY_SLOT_LOCAL_X[slot_index]
        return (px, py)

    def _lower_into_slot(self, slot_index, px, py):
        """Reach over the slot, set the rack down, let go and back off.

        Split out of place_in_slot so the sensors-only recovery above runs the
        IDENTICAL arm sequence rather than a parallel copy of it. Everything
        from here on is in base_link and does not care how px and py were
        arrived at.
        """
        log = self.get_logger()

        # NO CREEP. The delivery standoff is chosen so the slot is already inside
        # the arm's reach at this height, and _creep_forward could not be made to
        # work here anyway: measured twice, it published cmd_vel for its full
        # 25 s timeout and the base moved 0.000 m, while an external publisher on
        # the identical topic moved the same robot 0.71 m in six seconds. That is
        # unexplained and worth returning to, but it is not needed -- removing
        # the creep removes the dependency.
        #
        # WHAT A CLAMP COSTS HERE IS SMALL, unlike at a pick. If AMCL's error
        # puts the slot slightly beyond reach, placing at the cap sets the rack
        # down a few centimetres nearer the robot than the slot's centre. The
        # table is 0.668 m deep and the rack 0.16 m, with 0.15 m of lip in front
        # of the slot and 0.33 m behind, so a 0.05 m error still lands it
        # squarely on the top. Refusing outright, by contrast, means a robot
        # standing at the table holding a rack it will not put down.
        over_z = self.DELIVERY_PLACE_Z + BOX_SIZE / 2.0 + 0.04
        cap = self._max_x_for(py, over_z)
        if cap is None:
            log.error(f'[{self.ns}] slot {slot_index} is {py:+.3f} m to the side '
                      f'-- outside the arm envelope at any distance')
            return False
        if px > cap:
            log.warn(f'[{self.ns}] slot {slot_index} reads {px:.3f} m ahead at '
                     f'y={py:+.3f}; the arm reaches {cap:.3f} m there -- placing '
                     f'at the cap, {px - cap:.3f} m short')
            px = cap

        # Over the slot, then straight down onto the top. The over-height clears
        # the tray by the same margin a column placement uses.
        over_z = self.DELIVERY_PLACE_Z + BOX_SIZE / 2.0 + 0.04
        if not self.move_pose(px, py, over_z, 0.0, label=f'over-slot-{slot_index}',
                              quat_xyzw=zdown_quat(0.0), strict=True):
            log.error(f'[{self.ns}] could not reach over slot {slot_index} '
                      f'(rack still held)')
            return False
        # PRECISION, NOT TRANSIT. The reach OVER the slot above is free space
        # and runs at the transit scaling; this one sets a rack down between two
        # neighbours 0.14 m away on either side, at a pose computed from the
        # map -> base_link transform, so it keeps the slower scaling. See
        # slow_arm() in pick_and_place.py.
        with self.slow_arm(PRECISE_VEL, PRECISE_ACC):
            lowered = self.move_pose(px, py, self.DELIVERY_PLACE_Z, 0.0,
                                     cartesian=True,
                                     label=f'lower-into-slot-{slot_index}',
                                     quat_xyzw=zdown_quat(0.0))
        if not lowered:
            log.error(f'[{self.ns}] could not lower into slot {slot_index} '
                      f'(rack still held)')
            return False

        # DETACH *AND REMOVE* BEFORE THE RETREAT, NOT AFTER IT.
        #
        # detach_collision_object only stops the object riding with the gripper;
        # it stays in the scene as a WORLD object, at the pose it was released
        # at, which is exactly where the fingertips still are. Planning the
        # retreat from there is a start state in collision, and MoveIt refuses
        # it without planning. Measured on a run where the placement itself
        # worked:
        #
        #     [gripper] -> 0.038 release
        #     [arm] -> (0.74,0.17,0.43) cartesian retreat
        #     Action 'execute_trajectory' was unsuccessful: STATUS_ABORTED.
        #     [arm] motion failed: retreat
        #
        # The rack was already down and let go, so the errand survived it, but
        # the arm then backed away from the table on the HOME_CONFIG move
        # instead of the controlled vertical retreat -- over a rack it had just
        # set down between two neighbours. Removing the object first is what
        # makes the retreat plannable; the real rack is a Gazebo model and is
        # unaffected either way.
        self.arm.detach_collision_object(BOX_ID)
        self.arm.remove_collision_object(BOX_ID)
        time.sleep(SCENE_SYNC_RELEASE)
        # gripper() publishes the DetachableJoint release on any opening move,
        # so this both opens the jaws and breaks the weld.
        self.gripper(GRIP_OPEN, 'release')
        self.move_pose(px, py, over_z, 0.0, cartesian=True, label='retreat',
                       quat_xyzw=zdown_quat(0.0))

        # ORDER MATTERS, exactly as it does at a column: back the BASE off first
        # and only then bring the arm home, or the ready pose sweeps the open
        # jaws back down through the rack that was just placed.
        log.info(f'[{self.ns}] backing off the delivery table')
        self._drive_blind(-0.25, 4.0)
        self._stop_base()
        self.move_config(HOME_CONFIG, 'gripper-down ready')
        return True

    # --- the whole errand ----------------------------------------------------
    def _retreat_from_delivery(self):
        """Back off the delivery table and stand the arm up.

        A robot that fails AT the delivery table used to just stop there, still
        holding its rack, parked squarely on the one standoff the next robot
        needs. Measured: r2 failed its placement and sat on the spot; r1 was
        then granted the table, drove in, and the two ended 0.94 m apart -- the
        lock had done its job and the geometry undid it. Failing is allowed;
        occupying the shared resource afterwards is not.

        A 0.35 m BLIND REVERSE IS NOT LEAVING, and that is what this used to do.
        The manager releases the table the instant this returns and the status
        goes to 'failed'; measured on a later run, r3 was granted the table
        1.6 s after r2 gave up, while r2 was still within a metre of it and
        still reversing on an open-loop command with nothing watching behind.
        The two collided. The lock is only worth anything if "released" means
        the previous robot is GONE, so this now drives back to the holding ring
        -- a real Nav2 goal 3 m from the table, on the side the robot came from
        -- and only returns once it is there or has genuinely failed to get
        there.

        THE BLIND REVERSE IS STILL FIRST, and deliberately so: the robot may be
        close enough to the table that a planner refuses to start, and getting
        the footprint out of the inscribed zone before Nav2 is asked for
        anything is what makes the goal below plannable. Note that
        _drive_blind's arguments are a VELOCITY and a DURATION, so (-0.35, 4.0)
        is about 1.4 m of reverse, not 0.35 -- see _creep_blind.
        """
        log = self.get_logger()
        try:
            self._drive_blind(-0.35, 4.0)
            self._stop_base()
            self.move_config(HOME_CONFIG, 'gripper-down ready')
        except Exception as exc:
            log.warn(f'[{self.ns}] retreat failed: {exc}')
        try:
            here = self._base_xy_in_map()
            if here is None:
                log.warn(f'[{self.ns}] no pose to retreat from -- staying put')
                return
            hold = RT.delivery_hold_pose(here)
            log.info(f'[{self.ns}] clearing the delivery table back to the '
                     f'holding ring at ({hold[0]:+.2f},{hold[1]:+.2f})')
            if not self._navigate_with_recovery(hold, 'the holding ring',
                                                attempts=2):
                log.warn(f'[{self.ns}] could not get back to the holding ring '
                         f'-- the next robot is about to be sent to a table '
                         f'this one may still be near')
        except Exception as exc:
            log.warn(f'[{self.ns}] retreat to the holding ring failed: {exc}')

    def _navigate_with_recovery(self, pose, what, attempts=3, tight_yaw=False):
        """navigate_to, but treat a refusal as a POSE problem and re-localize.

        Smac 2D names the failure this exists for, in as many words:

            GridBased: failed to create plan, invalid use:
                Starting point in lethal space!

        The robot is not trapped -- it is standing in open floor next to a
        table -- but AMCL has placed it inside that table's inscribed zone, and
        a planner asked to start from a lethal cell has nothing to say. Every
        approach in this mission ends beside a table, so every approach can hit
        this, and a stopped robot cannot correct itself. See _relocalize.
        """
        for attempt in range(1, attempts + 1):
            if tight_yaw:
                self._set_yaw_goal_tolerance(TIGHT_YAW_TOLERANCE)
            ok = self.navigate_to(self.make_map_goal(*pose))
            if tight_yaw:
                self._set_yaw_goal_tolerance(DEFAULT_YAW_TOLERANCE)
            if ok:
                return True
            if attempt < attempts:
                self._relocalize(f'could not plan to {what}')
        return False

    def _base_xy_in_map(self, default=None):
        """This robot's (x, y) in the map frame, or `default` if TF is not there."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.tf_frame('base_link'), rclpy.time.Time(),
                timeout=RclDuration(seconds=2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'[{self.ns}] map <- base_link failed: {e}')
            return default
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _fleet_index(self):
        """This robot's position in ARM_ROBOTS, used to reserve one queue spot.

        Static assignment is deliberate: every robot runs exactly one errand,
        so there is nothing to contend over and no reason to make the manager
        arbitrate a third resource. It also means two robots can never be sent
        to the same waiting spot by a race.
        """
        from pickplace_arm_bringup.fleet_layout import ARM_ROBOTS
        try:
            return ARM_ROBOTS.index(self.ns)
        except ValueError:
            return 0

    def _relocalize(self, why):
        """Rotate in place until AMCL has had to re-weight against the laser.

        THE SINGLE ROOT CAUSE BEHIND BOTH FAILURE MODES IN THIS FILE, measured
        directly rather than inferred. With r3 stopped near the delivery table:

            AMCL believes r3 is at (-4.147, 10.461)
            ground truth (gz)        (-4.450,  9.252)
            discrepancy              1.25 m
            global costmap cost at AMCL pose = 99   <- inscribed, untraversable
            global costmap cost at true pose = 0    <- free

        NavFn plans from the pose AMCL reports, and it refuses to plan at all
        when the START cell is untraversable -- so every attempt came back
        "failed to create plan with tolerance 0.50" while the robot sat in
        open floor. The recovery behaviours cannot help either: backup checks
        the same poisoned pose and quits with "Collision Ahead". The robot is
        not stuck, it is misplaced, and nothing downstream can tell.

        AND IT CANNOT FIX ITSELF, because amcl only updates after update_min_d
        (0.12 m) or update_min_a (0.1 rad) of motion. A robot that has stopped
        somewhere wrong stays wrong: the same 1.75 m error was still there,
        unchanged to the millimetre, eleven minutes later.

        So the recovery has to supply the motion. Rotating is the cheapest
        motion that does it -- it clears update_min_a eight times over without
        translating anywhere the robot might not fit -- and the short reverse
        first is because this failure parks the robot hard against a table,
        which is exactly where a spin would grind.
        """
        log = self.get_logger()
        log.warn(f'[{self.ns}] relocalizing: {why}')
        self._drive_blind(-0.20, 2.5)
        for _ in range(8):
            self._rotate_step(math.pi / 4.0)
        self._stop_base()

    def _rack_in_view(self, colour, tries=6):
        """Can the front camera actually SEE the rack from where we stopped?

        This is the question claw_pick needs answered, and it is not the same
        question navigate_to answers -- see _search_for_rack."""
        self.move_config(HOME_CONFIG, 'gripper-down ready')
        for _ in range(tries):
            if self.detect_box_front(timeout_sec=0.5, color=colour) is not None:
                return True
        return False

    def _search_for_rack(self, colour):
        """Recovery for having stopped somewhere other than the standoff.

        WHY THIS IS NEEDED. Nav2 reports success against AMCL's ESTIMATE, not
        against the world. Measured on the run that motivated this: r1's filter
        sat 1.75 m from truth, so Nav2 stopped the robot wedged against the end
        of the collection bench -- 1.86 m from the standoff, pointing 29 deg
        off -- and reported the goal reached. claw_approach then called
        _face_box, which turns toward where the robot BELIEVES the rack is, so
        the camera swung to the wrong bearing and logged "only 0 valid red
        pixels" for eleven straight seconds before the errand gave up. The rack
        was sitting on its table, untouched, the entire time.

        A STATIONARY ROBOT CANNOT RECOVER BY WAITING. amcl only runs an update
        after update_min_d (0.12 m) or update_min_a (0.1 rad) of motion, so a
        robot parked in the wrong place keeps its wrong estimate forever: that
        1.75 m error was still exactly 1.75 m eleven minutes later, to the
        millimetre.

        Rotating is the cheapest thing that fixes both halves at once. It clears
        update_min_a on every step, so the filter finally gets to re-weight
        against the laser, and it sweeps the camera around the room, so the rack
        is found even while the filter is still wrong. The short reverse comes
        first because this failure parks the robot AGAINST the bench, and a spin
        started there just grinds along it.
        """
        log = self.get_logger()
        log.warn(f'[{self.ns}] no {colour} rack in view -- searching')
        self._drive_blind(-0.15, 2.0)
        found = self._sweep_for_rack(colour)
        if not found:
            log.warn(f'[{self.ns}] full turn without seeing the {colour} rack')
        return found

    # Sweep rate. Faster than SPIN_ANGULAR's 0.5 because nothing is being
    # measured precisely here -- the camera runs at 10 Hz, so at 0.7 rad/s
    # successive frames are 4 deg apart and a 0.06 m grip block spanning ~15 px
    # cannot fall between two of them.
    SWEEP_ANGULAR = 0.7

    def _sweep_for_rack(self, colour, sweep=2.0 * math.pi):
        """Rotate continuously, watching the camera, and stop facing the rack.

        WHY NOT _rotate_step. That turns 45 deg, STOPS, waits SPIN_SETTLE_SEC
        for the base to damp, and only then looks -- so a full turn costs
        8 x (1.57 s turn + 0.6 s settle + 0.5 s detect) = about 23 s, and the
        rack is only ever noticed at one of eight fixed bearings. It was written
        for a wrist camera that had to be still to capture cleanly; the front
        camera does not, and in simulation there is no motion blur to avoid.

        Sweeping while looking costs 10.5 s for the worst case (a full turn at
        0.7 rad/s) and typically far less, because it stops the moment the rack
        enters the frame -- which also leaves the base pointing AT it, so the
        servo that follows starts centred instead of re-acquiring.
        """
        log = self.get_logger()
        twist = Twist()
        twist.angular.z = self.SWEEP_ANGULAR
        turned = 0.0
        t_prev = time.time()
        while turned < sweep and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            det = self.detect_box_front(timeout_sec=0.15, color=colour)
            now = time.time()
            turned += self.SWEEP_ANGULAR * (now - t_prev)
            t_prev = now
            if det is not None:
                self._stop_base()
                # Let the chassis stop rocking before the servo takes over.
                time.sleep(0.4)
                log.info(f'[{self.ns}] {colour} rack found after '
                         f'{math.degrees(turned):.0f} deg of sweep')
                return True
        self._stop_base()
        return False

    def run_errand(self, colour, table, rack_xy, stand):
        log = self.get_logger()

        # Point the inherited layout attributes at THIS errand. claw_pick reads
        # them, so this is what makes one mission class serve three different
        # collection tables.
        self.LAYOUT_TABLE_APPROACH = stand
        self.LAYOUT_TABLE_GRASP_Z = GROUND_Z + RT.TABLE_TOP + RT.RACK_GRIP_HEIGHT

        self.say('collecting', f'{colour} at ({rack_xy[0]:.2f},{rack_xy[1]:.2f})')
        # ARRIVAL IS VERIFIED AGAINST THE CAMERA, NOT AGAINST NAV2. "Goal
        # reached" is a claim about where AMCL thinks the robot is; having the
        # rack in frame is a claim about where it actually is, and only the
        # second one is what claw_pick needs. A navigate_to that succeeds and
        # leaves nothing in view is therefore treated as a failed approach and
        # retried, because that is exactly the shape of the bug that cost a
        # full errand: see _search_for_rack.
        reached = False
        for attempt in range(1, 4):
            self._set_yaw_goal_tolerance(TIGHT_YAW_TOLERANCE)
            ok = self.navigate_to(self.make_map_goal(*stand))
            self._set_yaw_goal_tolerance(DEFAULT_YAW_TOLERANCE)
            if ok and self._rack_in_view(colour):
                reached = True
                break
            if attempt < 3:
                # The spin re-converges AMCL as much as it finds the rack, so
                # the re-issued goal is planned from a corrected pose.
                self._search_for_rack(colour)
        if not reached:
            return self._fail('could not reach the collection table')

        # face_first=False: the approach loop above only exits once the camera
        # has the rack in frame, so re-aiming from the map estimate can only
        # make it worse. See claw_approach.
        if not self.claw_pick(rack_xy, color=colour,
                              grasp_z=self.LAYOUT_TABLE_GRASP_Z,
                              x_offset=self.LAYOUT_TABLE_X_OFFSET,
                              face_first=False):
            return self._fail(f'could not pick the {colour} rack')
        self.say('carrying', colour)

        # Get clear of the table before turning: the bench is right in front of
        # the robot and Nav2's collision check will refuse to plan through it.
        self._drive_blind(-0.20, 3.5)
        self._stop_base()

        # DRIVE TO THE DELIVERY TABLE FIRST AND WAIT THERE, then ask for the
        # table. All three robots make the trip as soon as they have their rack;
        # they queue at the table and go in one at a time.
        #
        # WHERE THE WAIT HAPPENS IS THE WHOLE POINT. Only one robot can use the
        # standoff -- it is barely wider than one robot, and the arm works
        # across all three slots from it. An earlier version took the lock
        # BEFORE setting off, which made the approach exclusive but also meant
        # the two robots that lost the race simply stood at their collection
        # tables, half a building away, until the winner had finished. The
        # errands were concurrent right up to the last leg and then serialised
        # completely.
        #
        # Waiting on a reserved spot 1.4 m behind the standoff keeps both
        # properties: the approach is still exclusive, because a robot only
        # leaves its spot once it holds the lock, but all three trips overlap.
        # The spots are abreast rather than in single file precisely so the
        # grant order does not have to match the queue order -- see
        # rack_table_layout.delivery_queue().
        # HOLD AT A DISTANCE, WHEREVER THE APPROACH PUTS US. The target is a
        # point on a circle of RT.DELIVERY_HOLD_RADIUS around the table, on the
        # line from wherever this robot is now -- so it stops on its way in
        # rather than crossing the front of the table to reach an assigned spot.
        here = self._base_xy_in_map(default=rack_xy)
        hold_pose = RT.delivery_hold_pose(here)
        self.say('approaching', 'the delivery table')
        if not self._navigate_with_recovery(hold_pose, 'the holding ring'):
            # Not fatal: the robot has stopped somewhere short of the ring,
            # still holding its rack, and the manager's ordering below is what
            # actually keeps the standoff exclusive.
            self.get_logger().warn(
                f'[{self.ns}] could not reach the holding ring -- waiting '
                f'where it stands')
        self._stop_base()
        # SAID ONLY ONCE THE ROBOT HAS STOPPED. The manager holds every robot
        # here until ALL of them are holding, so announcing this early would
        # release the fleet while somebody was still driving.
        self.say('holding', f'{RT.DELIVERY_HOLD_RADIUS:.1f} m from the table')

        # 1800 s, up from 900. The wait is now genuinely serial: this robot
        # holds until every robot has reached the ring, and then until every
        # robot ahead of it in fleet order has PLACED AND PARKED. Three full
        # turns plus their recoveries can exceed fifteen minutes, and timing out
        # here would fail a robot that was waiting exactly as designed.
        slot = self._claim(self._slot_client, 'slot', wait_sec=1800.0)
        if slot is None:
            return self._fail('no delivery slot available')
        slot_xy = RT.delivery_slots()[slot][1]
        self.say('delivering', f'slot {slot}')

        # SAME RECOVERY AS THE COLLECTION APPROACH, for the same reason: a
        # planner that cannot plan out of open floor is reporting a bad pose,
        # not a blocked route, and only motion fixes that. See _relocalize.
        # It goes through _drive_to_delivery_standoff rather than
        # _navigate_with_recovery because for THIS goal Nav2 has been measured
        # to report success without moving the robot -- see there.
        if not self._drive_to_delivery_standoff():
            # RETREAT EVEN THOUGH WE NEVER ARRIVED. Whatever went wrong, the
            # robot is somewhere near the delivery table still holding its
            # rack, and the manager is about to hand the table to someone else.
            # _retreat_from_delivery's docstring describes this exact cascade
            # being fixed for the PLACEMENT failure; the navigation failure
            # reaches the same end state and was left out. Measured with it
            # missing: r3 gave up and stopped 1.5 m from the table, r1 was
            # granted the table next and could not plan past it, then r2 could
            # not either -- one bad pose cost all three racks.
            self._retreat_from_delivery()
            return self._fail('could not reach the delivery table')

        # The rack can slip out during the carry, and nothing downstream would
        # notice: the placement would lower an empty gripper and report success.
        # grasp_is_holding is a finger-gap read, so this costs nothing.
        if not self.grasp_is_holding():
            self._retreat_from_delivery()
            return self._fail(f'dropped the {colour} rack during the carry')
        if not self.place_in_slot(slot, slot_xy):
            self._retreat_from_delivery()
            return self._fail(f'could not place in slot {slot}')
        self.say('delivered', f'slot {slot}')
        return True

    # --- clearing the floor -------------------------------------------------
    #
    # PARKING IS NOT PART OF THE ERRAND, AND MAKING IT PART OF THE ERRAND WAS
    # THE BUG. It used to be the tail of run_errand, which meant every one of
    # the six failure paths above skipped it: a robot that failed stopped dead
    # exactly where it failed. Measured on the 16:38 run, r1 could
    # not place in slot 0 and r3 could not reach the delivery standoff, and
    # NEITHER ever asked the manager for a vertex -- only r2, the one robot
    # whose errand succeeded, ever parked. Two thirds of the fleet spent the
    # rest of the run stopped in the lobby, one of them still holding its rack
    # in front of the delivery table the other robots were queuing for.
    #
    # A robot that has finished is in the way, and whether it finished WELL has
    # nothing to do with that. So run() calls this however the errand ended.
    def _park(self, outcome, detail):
        """Drive to a parking vertex and stop, then restate the errand outcome.

        `outcome`/`detail` are how the errand actually ended. They are passed in
        rather than recomputed because the LAST status a robot publishes is what
        the manager's closing report prints, and parking must not be allowed to
        overwrite a failure reason with a cheerful one -- the report would then
        credit a robot whose rack is still on its collection table. Same trap
        the 'done'-unconditionally bug fell into; see run().
        """
        # ANNOUNCED BEFORE THE DRIVE, NOT AFTER. The manager frees the delivery
        # table on 'done'/'failed' (see _on_status), so saying the outcome first
        # lets the next robot start placing while this one is still crossing the
        # lobby. Parking behind the terminal status instead serialised the whole
        # fleet on one robot's drive.
        self.say(outcome, detail)

        vertex = self._claim(self._park_client, 'park')
        if vertex is None:
            # Not a failure of the errand -- the errand already ended, and its
            # outcome stands. The robot simply stays where it stopped.
            self.get_logger().warn(
                f'[{self.ns}] no parking vertex available -- staying put')
            return
        from pickplace_arm_bringup.fleet_layout import parking_vertices
        pose = parking_vertices()[vertex]
        self.say('parking', f'vertex {vertex}')
        # Same recovery as every other approach here. The parking drive starts
        # AT the delivery table, which is exactly the place a robot's pose ends
        # up inside an inscribed zone, so this leg fails the same way the others
        # do -- measured: r1 placed its rack correctly and was then recorded as
        # "failed could not reach parking vertex 1".
        #
        # ALWAYS LEAVES A TERMINAL STATUS BEHIND IT, on every path including the
        # exception one. all_done() treats 'parking' as still-running and the
        # manager's wait loop has no timeout, so a robot that died mid-drive
        # would hang the closing report forever.
        try:
            if self._navigate_with_recovery(pose, f'parking vertex {vertex}'):
                note = f'{detail} -- parked at vertex {vertex}'.lstrip(' -')
            else:
                note = f'{detail} -- could not reach parking vertex {vertex}'.lstrip(' -')
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(f'[{self.ns}] parking drive raised: {exc}')
            note = f'{detail} -- parking drive failed'.lstrip(' -')
        # The OUTCOME is unchanged: a delivered rack stays delivered whether or
        # not the robot reached its vertex, and a failed errand stays failed.
        self.say(outcome, note)

    def run(self):
        """Wait for a task, run it, then idle. One errand per robot is what this
        plan asks for; a second task published later would simply be picked up."""
        # PATIENT ON PURPOSE. The default 60 s is tuned for a single robot whose
        # AMCL is up before the mission node exists. In a fleet the stacks come
        # up staggered and the last one can be minutes behind the first, so a
        # short wait reports "never localized" for a robot that was simply
        # queued. This is also why the launch gate no longer checks the
        # transform itself -- see hospital_mission.launch.py.
        if not self.wait_for_localization(timeout_sec=600.0):
            self.say('failed', 'never localized')
            return
        self.say('idle', 'waiting for a task')
        while rclpy.ok():
            task = self.take_task()
            if task is None:
                time.sleep(0.5)
                continue
            # colour|table_x,table_y,table_yaw|rack_x,rack_y|stand_x,stand_y,stand_yaw
            try:
                parts = task.split('|')
                colour = parts[0]
                table = tuple(float(v) for v in parts[1].split(','))
                rack = tuple(float(v) for v in parts[2].split(','))
                stand = tuple(float(v) for v in parts[3].split(','))
            except (IndexError, ValueError) as e:
                self.get_logger().error(f'[{self.ns}] unreadable task {task!r}: {e}')
                continue
            # A FAILED ERRAND KEEPS ITS 'failed' STATUS AND ITS REASON. Saying
            # 'done' unconditionally overwrote the reason with an empty string,
            # so the manager's closing report announced that every robot had
            # finished while one rack was still sitting on its collection
            # table. 'failed' is already terminal for all_done(), so the fleet
            # still shuts down cleanly -- it just stops lying about how.
            self._fail_reason = ''
            if self.run_errand(colour, table, rack, stand):
                outcome, detail = 'done', ''
            else:
                outcome, detail = 'failed', self._fail_reason
            # PARKS EITHER WAY. _park publishes the outcome above and only then
            # drives, so a robot that failed still clears the floor instead of
            # standing where it broke. See _park.
            self._park(outcome, detail)
            # ONE ERRAND PER ROBOT, which is what this plan asks for. Returning
            # rather than looping also stops a failed errand being retried
            # instantly and forever against a machine that is already the reason
            # it failed.
            return


def main():
    rclpy.init()
    node = DeliveryMission()
    ex = rclpy.executors.MultiThreadedExecutor(4)
    ex.add_node(node)

    def task():
        time.sleep(3.0)
        node.run()

    threading.Thread(target=task, daemon=True).start()
    try:
        # rclpy intermittently raises RCLError out of spin() under load; left
        # uncaught it kills the executor and the mission thread hangs forever.
        while rclpy.ok():
            try:
                ex.spin()
                break
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                node.get_logger().warn(f'[executor] recovered: {exc}')
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
