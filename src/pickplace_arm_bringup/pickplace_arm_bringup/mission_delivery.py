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

import rclpy
import tf2_ros
from rclpy.duration import Duration as RclDuration
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String
from std_srvs.srv import Trigger

from pickplace_arm_bringup.mission_2 import Mission2Hospital, TIGHT_YAW_TOLERANCE, \
    DEFAULT_YAW_TOLERANCE
from pickplace_arm_bringup.pick_and_place import (
    BOX_ID, BOX_SIZE, GRIP_OPEN, GROUND_Z, HOME_CONFIG, zdown_quat)
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
    # 0.93 of full extension. Seeded IK starts failing well before 1.0, and the
    # cost of being conservative is a few centimetres of placement error on a
    # table 0.668 m deep.
    ARM_REACH_FRACTION = 0.93

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
        log = self.get_logger()
        time.sleep(2.0)
        deadline = time.time() + timeout_sec
        prev = None
        while time.time() < deadline:
            cur = self._slot_in_base_link(slot_xy)
            if cur is None:
                time.sleep(0.5)
                continue
            if prev is not None and math.hypot(cur[0] - prev[0],
                                               cur[1] - prev[1]) < 0.03:
                return cur
            prev = cur
            time.sleep(1.0)
        if prev is not None:
            log.warn(f'[{self.ns}] slot reading never settled -- using the last '
                     f'({prev[0]:+.3f},{prev[1]:+.3f})')
        return prev

    def _close_in_on(self, slot_xy, px):
        """Drive forward so the slot lands inside the arm's reach, using NAV2.

        The obvious tool is _creep_forward, which measures on odom and is what
        the column placement uses. It does not work here: measured twice, it
        published cmd_vel for its full 25 s timeout and the base moved 0.000 m,
        while an external publisher on the identical topic moved the same robot
        0.71 m in six seconds. The publisher is matched (ros2 topic info -v
        lists this node on that topic, QoS compatible) so the cause is still
        open -- but the fix does not have to wait for the explanation.

        Nav2 demonstrably drives this robot, and it has the costmap, so it will
        not push the chassis into the table the way an open-loop creep could.
        The goal is the robot's own believed pose advanced along its own
        heading by exactly the shortfall.
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
        # at it, so AMCL's own error does not put it back outside.
        advance = px - 0.78
        gx = t.x + advance * math.cos(yaw)
        gy = t.y + advance * math.sin(yaw)
        log.info(f'[{self.ns}] slot is {px:.3f} m ahead -- moving up '
                 f'{advance:.2f} m to ({gx:+.2f},{gy:+.2f})')
        return self.navigate_to(self.make_map_goal(gx, gy, yaw))

    def place_in_slot(self, slot_index, slot_xy):
        """Put the carried rack down in `slot_xy` (map frame) and let go."""
        log = self.get_logger()
        target = None
        for attempt in range(3):
            target = self._settled_slot(slot_xy)
            if target is None:
                return False
            # Checked against the OVER-slot waypoint, which is the higher and
            # therefore tighter of the two poses the arm has to make.
            over_z = self.DELIVERY_PLACE_Z + BOX_SIZE / 2.0 + 0.04
            cap = self._max_x_for(target[1], over_z)
            if cap is not None and target[0] <= cap:
                break
            # OUT OF REACH IS RECOVERABLE, and worth recovering rather than
            # clamping. Nav2 stops within its own 0.20 m tolerance and AMCL
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
        log.info(f'[{self.ns}] slot {slot_index} is at base_link '
                 f'({px:+.3f},{py:+.3f})')

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
        if not self.move_pose(px, py, self.DELIVERY_PLACE_Z, 0.0, cartesian=True,
                              label=f'lower-into-slot-{slot_index}',
                              quat_xyzw=zdown_quat(0.0)):
            log.error(f'[{self.ns}] could not lower into slot {slot_index} '
                      f'(rack still held)')
            return False

        self.arm.detach_collision_object(BOX_ID)
        time.sleep(0.3)
        # gripper() publishes the DetachableJoint release on any opening move,
        # so this both opens the jaws and breaks the weld.
        self.gripper(GRIP_OPEN, 'release')
        self.move_pose(px, py, over_z, 0.0, cartesian=True, label='retreat',
                       quat_xyzw=zdown_quat(0.0))
        self.arm.remove_collision_object(BOX_ID)

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
        """
        try:
            self._drive_blind(-0.35, 4.0)
            self._stop_base()
            self.move_config(HOME_CONFIG, 'gripper-down ready')
        except Exception as exc:
            self.get_logger().warn(f'[{self.ns}] retreat failed: {exc}')

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
        for step in range(8):
            self._rotate_step(math.pi / 4.0)
            if self.detect_box_front(timeout_sec=0.5, color=colour) is not None:
                log.info(f'[{self.ns}] {colour} rack found after '
                         f'{(step + 1) * 45} deg of search')
                return True
        log.warn(f'[{self.ns}] full turn without seeing the {colour} rack')
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
            self.say('failed', 'could not reach the collection table')
            return False

        if not self.claw_pick(rack_xy, color=colour,
                              grasp_z=self.LAYOUT_TABLE_GRASP_Z,
                              x_offset=self.LAYOUT_TABLE_X_OFFSET):
            self.say('failed', f'could not pick the {colour} rack')
            return False
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
            self.say('failed', 'no delivery slot available')
            return False
        slot_xy = RT.delivery_slots()[slot][1]
        self.say('delivering', f'slot {slot}')

        stand_d = RT.delivery_standoff()
        # SAME RECOVERY AS THE COLLECTION APPROACH, for the same reason: a
        # planner that cannot plan out of open floor is reporting a bad pose,
        # not a blocked route, and only motion fixes that. See _relocalize.
        if not self._navigate_with_recovery(stand_d, 'the delivery table',
                                            tight_yaw=True):
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
            self.say('failed', 'could not reach the delivery table')
            return False

        # The rack can slip out during the carry, and nothing downstream would
        # notice: the placement would lower an empty gripper and report success.
        # grasp_is_holding is a finger-gap read, so this costs nothing.
        if not self.grasp_is_holding():
            self._retreat_from_delivery()
            self.say('failed', f'dropped the {colour} rack during the carry')
            return False
        if not self.place_in_slot(slot, slot_xy):
            self._retreat_from_delivery()
            self.say('failed', f'could not place in slot {slot}')
            return False
        self.say('delivered', f'slot {slot}')

        vertex = self._claim(self._park_client, 'park')
        if vertex is None:
            self.say('failed', 'no parking vertex available')
            return False
        from pickplace_arm_bringup.fleet_layout import parking_vertices
        pose = parking_vertices()[vertex]
        self.say('parking', f'vertex {vertex}')
        # Same recovery as every other approach here. The parking drive starts
        # AT the delivery table, which is exactly the place a robot's pose ends
        # up inside an inscribed zone, so this leg fails the same way the others
        # do -- measured: r1 placed its rack correctly and was then recorded as
        # "failed could not reach parking vertex 1".
        if not self._navigate_with_recovery(pose, f'parking vertex {vertex}'):
            self.say('failed', f'could not reach parking vertex {vertex}')
            return False
        self.say('parked', f'vertex {vertex}')
        return True

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
            if self.run_errand(colour, table, rack, stand):
                self.say('done', '')
            # A FAILED ERRAND KEEPS ITS 'failed' STATUS AND ITS REASON. Saying
            # 'done' unconditionally overwrote the reason with an empty string,
            # so the manager's closing report announced that every robot had
            # finished while one rack was still sitting on its collection
            # table. 'failed' is already terminal for all_done(), so the fleet
            # still shuts down cleanly -- it just stops lying about how.
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
