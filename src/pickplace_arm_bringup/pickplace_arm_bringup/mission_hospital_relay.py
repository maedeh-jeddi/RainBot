#!/usr/bin/env python3
"""STAGE 5: the sample run as a two-robot relay.

mission_hospital_aws.py drives the whole south-wing-to-lobby route with one
robot, and says in its own docstring why: every part of that route except the
handover is shared with the relay, so getting it working once means the relay is
that route cut in half with a handover spliced into the join. This is that cut.

    r1, the CARRIER      ring -> collect bench -> handover -> ring
    r2, the RECEIVER     ring -> handover -> delivery dock -> ring

Neither robot drives the whole route, and neither of them runs a line of
mission logic that the single-robot version did not already run: the carrier's
leg is Mission2.collect_from_table, the receiver's is
Mission2.deliver_to_column, and both were split out of run_mission_2 unchanged
for exactly this. What is new is the join - the handshake in handover.py, and
the set-down / pick-up pair below.

WHY TWO NODES AND NOT ONE COORDINATOR. Everything in this package is built
per-robot and namespaced: the TF tree, the cameras, the move_group, the Nav2
stack. A single node driving both robots would need two of every client, two TF
prefixes and two MoveIt models inside one process, and would defeat the
namespacing the whole fleet is built on. Two mission nodes, one per namespace,
each identical in shape to the single-robot mission, talk over the one thing
they deliberately share (see handover.py).

WHAT THE RELAY DOES NOT SOLVE. The two robots still avoid each other only
through their own LIDAR and local costmaps - there is no traffic control, and
the fleet launch file says why that is a later stage. The protocol keeps them
apart in TIME instead, which is enough here: the receiver does not set off until
the carrier has the rack (by which point the carrier is in the south wing,
18.5 m away), and the carrier does not drive home until the receiver has picked
the rack up and can move out of its way. The one moment they are close is the
handover itself, where they stand 1.55 m apart facing each other, which is the
pose pair hospital_aws_layout measured for it.
"""
import math
import time

from pickplace_arm_bringup.handover import CARRIER, HandoverPeer, RECEIVER
from pickplace_arm_bringup.mission import CLAW_STOP_X as OPEN_CLAW_STOP_X
from pickplace_arm_bringup.mission_2 import (
    DEFAULT_YAW_TOLERANCE, TIGHT_YAW_TOLERANCE, _run)
from pickplace_arm_bringup.mission_hospital_aws import MissionHospitalAws
from pickplace_arm_bringup.pick_and_place import (
    BOX_ID, GRIP_OPEN, HOME_CONFIG, LIFT_CLEARANCE, MAX_REACH_X, zdown_quat)

from pickplace_arm_bringup.hospital_aws_layout import (  # noqa: E402
    HANDOVER_CARRIER, HANDOVER_POINT, HANDOVER_RECEIVER, HANDOVER_VIA,
    RELAY_CARRIER_NS, RELAY_RECEIVER_NS, ROBOTS,
)

# --- who is who ---------------------------------------------------------------
# Both are ARM_ROBOTS and NAV_ROBOTS already; the other two of the fleet are
# neither, so the relay cannot be run by another pair without changing those
# lists first (and reading the CPU measurements in aws_hospital_fleet.launch.py
# before doing so).
CARRIER_NS = RELAY_CARRIER_NS
RECEIVER_NS = RELAY_RECEIVER_NS
# Each robot parks back on its own spot in the ring, so a repeat run starts
# exactly where the first one did. Read out of the fleet list rather than
# written twice: the ring is fixed and duplicating it is how the two copies
# drift apart.
_RING = {ns: (x, y, yaw) for ns, x, y, yaw in ROBOTS}
CARRIER_PARK = _RING[CARRIER_NS]
RECEIVER_PARK = _RING[RECEIVER_NS]

# How far ahead of a robot standing at its meet pose the transfer point sits.
# Derived from the layout rather than written down, because it is a property of
# the two poses: the point is their midpoint, so this is half their separation.
# It comes out at 0.775 m - inside MAX_REACH_X (0.85) with 0.075 m to spare,
# which is the margin map_point_in_base exists to protect.
HANDOVER_REACH_X = math.hypot(HANDOVER_POINT[0] - HANDOVER_CARRIER[0],
                              HANDOVER_POINT[1] - HANDOVER_CARRIER[1])

# --- how long each robot waits for the other ----------------------------------
#
# GENEROUS ON PURPOSE, and for the same reason the launch gates are (see the
# timeout note in mission_hospital_aws.launch.py): the legs these cover really
# do take minutes, and a wait that expires early does not fail safely - it makes
# one robot give up on a partner that was simply still driving. The protocol
# already handles the case that matters, a peer that fails and says so, without
# using any of this budget at all.
#
# The collect leg is the long one: fleet bring-up, then a ~45 m drive round the
# west corridor (NAV_TIMEOUT_SEC is 300 s for that single goal), then a full
# visual-servo pick with up to three attempts.
WAIT_FOR_COLLECT = 1800.0
# Everything after that is a single drive plus a single manipulation.
WAIT_AT_HANDOVER = 900.0


class RelayCarrier(HandoverPeer, MissionHospitalAws):
    """r1: collect the rack in the south wing, leave it at the handover point.

    Inherits the whole AWS hospital layout - the collect bench, the rack, the
    shelf overhang, the detection gate - and replaces only the second half of
    the route.
    """

    ROLE = CARRIER
    LAYOUT_FINAL_POSE = CARRIER_PARK

    # --- putting the rack down ----------------------------------------------
    def set_rack_down(self):
        """Stand the rack on the floor at the handover point and let go.

        This is place_on_column's lower/detach/release/retreat with the target
        height set to the floor, and it is deliberately the same shape: every
        move is checked, and nothing is detached or released until the position
        it would be released AT has been confirmed. A failed move that goes
        unchecked leaves the arm wherever it stopped, and opening the jaws there
        drops the rack somewhere the receiver will never find it - while every
        step still logs success. That failure has already happened once in this
        package (see the comment in place_on_column) and it is worth not
        repeating in a place where a second robot is waiting on the result.

        WHERE, EXACTLY, IS MEASURED AND NOT ASSUMED. See map_point_in_base: Nav2
        parks with a 0.20 m tolerance and the whole margin between the nominal
        0.775 m transfer point and the arm's 0.85 m reach cap is 0.075 m.
        """
        log = self.get_logger()
        target = self.map_point_in_base(*HANDOVER_POINT)
        if target is None:
            log.error('[relay] cannot locate the handover point in base_link')
            return False
        px, py = target
        log.info(f'[relay] handover point is at base_link ({px:.3f},{py:+.3f}) '
                 f'(nominal {HANDOVER_REACH_X:.3f})')

        # PARKED SHORT? DRIVE THE DIFFERENCE, DO NOT CLAMP IT. Clamping px to
        # MAX_REACH_X and placing anyway is the exact mistake place_on_column
        # documents for COLUMN_DEPTH_BIAS: the arm stops at its reach cap and
        # releases the payload into thin air short of the target, reporting
        # success. _creep_forward closes the gap on odom instead, and it is
        # measured rather than timed.
        if px > MAX_REACH_X:
            creep = px - HANDOVER_REACH_X
            log.info(f'[relay] parked {creep:.2f} m short of the meet pose -- '
                     f'creeping forward before setting the rack down')
            if not self._creep_forward(creep):
                log.error('[relay] creep fell short -- not setting the rack '
                          'down out of reach (still held)')
                return False
            target = self.map_point_in_base(*HANDOVER_POINT)
            if target is None:
                return False
            px, py = target
            log.info(f'[relay] re-measured: base_link ({px:.3f},{py:+.3f})')

        # Anything outside this window means the robot is not where the protocol
        # believes it is, and putting a payload down on that belief is worse
        # than keeping hold of it: the receiver would servo onto empty floor.
        if not (0.55 <= px <= MAX_REACH_X) or abs(py) > 0.25:
            log.error(f'[relay] handover point at ({px:.2f},{py:+.2f}) is not '
                      f'where a robot at the meet pose should see it -- '
                      f'refusing to set the rack down (still held)')
            return False

        # The fingertips hold the grip block, which sits PAYLOAD_GRIP_HEIGHT
        # above the rack's own tray - so this is exactly the height the
        # fingertips must be at for that tray to be resting on the floor, which
        # is what PAYLOAD_FLOOR_Z already means (it is what _placement_landed
        # compares against to tell "on the target" from "on the floor").
        set_z = self.PAYLOAD_FLOOR_Z
        over_z = set_z + LIFT_CLEARANCE
        log.info(f'=== RELAY: setting the rack down at ({px:.2f},{py:+.2f}) ===')
        # strict: a failed direct move must never fall back to an unseeded pose
        # plan while holding the rack -- that can pick any IK solution, and this
        # payload is a pendulum on a 0.17 m gantry.
        if not self.move_pose(px, py, over_z, 0.0, label='over-transfer-point',
                              quat_xyzw=zdown_quat(0.0), strict=True):
            log.error('[relay] could not reach over the handover point -- '
                      'aborting (rack still held)')
            return False
        if not self.move_pose(px, py, set_z, 0.0, cartesian=True,
                              label='lower-to-floor', quat_xyzw=zdown_quat(0.0)):
            log.error('[relay] could not lower the rack -- aborting (still held)')
            return False
        self.arm.detach_collision_object(BOX_ID)
        time.sleep(0.3)
        # Opening the jaws breaks the weld too: detach is hooked on gripper-open
        # in PickAndPlace.gripper precisely so no call site can forget it.
        self.gripper(GRIP_OPEN, 'release at the handover point')
        self.move_pose(px, py, over_z, 0.0, cartesian=True, label='retreat',
                       quat_xyzw=zdown_quat(0.0))
        self.arm.remove_collision_object(BOX_ID)
        log.info('=== RELAY: rack is on the floor ===')
        return True

    def withdraw(self):
        """Back out of the receiver's working space and stop.

        ORDER MATTERS, exactly as it does at the end of place_on_column: the
        base moves first and the arm goes home second. HOME_CONFIG holds the
        fingertips 0.50 m above the floor at x=0.70, and the rack now stands at
        x~0.775 with its grip block at 0.20 - so returning to the ready pose
        while still parked over it sweeps the open jaws down beside a payload
        that is no longer welded to anything.

        _drive_blind takes a SPEED and a DURATION, not a distance: 0.15 m/s for
        4 s is about 0.60 m, inside the 0.80 m of reverse clearance the layout
        measured behind this pose. Blind is safe here for the one reason it was
        not safe at the collect bench - the robot has not been swung off its
        approach heading by a visual servo, so what is behind it is the corridor
        it drove in along.
        """
        self.get_logger().info('[relay] backing out of the handover space')
        self._drive_blind(-0.15, 4.0)
        self._stop_base()
        self.move_config(HOME_CONFIG, 'gripper-down ready')

    # --- the leg ------------------------------------------------------------
    def run_mission_2(self):
        log = self.get_logger()
        log.info('=== RELAY CARRIER: START ===')
        if not self.wait_for_localization():
            self.announce_failure('never localized')
            return
        self.announce('start')

        color, box_xy = self.LAYOUT_BOXES[0]
        if not self.collect_from_table(color, box_xy):
            self.announce_failure('collect leg failed')
            return
        self.announce('collected')

        log.info('=== RELAY CARRIER: driving to the handover ===')
        # VIA THE CORRIDOR WAYPOINT, NOT STRAIGHT AT THE MEET POSE. Sent
        # straight, this leg drove into a dead-end pocket and wedged the robot
        # on two runs out of two; see HANDOVER_VIA for the measurements and for
        # why the global planner is willing to route through a gap the robot
        # does not fit in.
        #
        # Left on the DEFAULT yaw tolerance deliberately. These are transit
        # goals - nothing is measured relative to them - and the loose tolerance
        # is what keeps this skid-steer base from oscillating on arrival and
        # tripping its own Spin recovery.
        for i, wp in enumerate(HANDOVER_VIA, 1):
            log.info(f'[relay] via waypoint {i}/{len(HANDOVER_VIA)} '
                     f'({wp[0]:.2f},{wp[1]:.2f})')
            if not self.navigate_to(self.make_map_goal(*wp)):
                self.announce_failure(
                    f'could not reach handover waypoint {i} ({wp[0]:.2f},{wp[1]:.2f})')
                return
        # Tightened for the same reason every approach goal in this package is:
        # the arm acts on a point measured relative to this pose, and an
        # off-axis arrival moves that point sideways under the gripper.
        self._set_yaw_goal_tolerance(TIGHT_YAW_TOLERANCE)
        nav_ok = self.navigate_to(self.make_map_goal(*HANDOVER_CARRIER))
        self._set_yaw_goal_tolerance(DEFAULT_YAW_TOLERANCE)
        if not nav_ok:
            self.announce_failure('could not reach the handover pose')
            return
        # The same arrival check the delivery leg makes, and worth being precise
        # about what it does and does not catch here. Once the rack is WELDED,
        # grasp_is_holding short-circuits to True - the finger gap stops meaning
        # anything the moment the weld disables box/finger collision, which is
        # documented at that method. So this catches the paths that released the
        # rack and cleared the weld on their way here (a failed carry move, a
        # slip caught before the weld, a retry that opened the jaws), not a
        # physical drop of a welded payload - which the weld is there to make
        # impossible. It is a cheap guard against handing over nothing at all,
        # which would leave the receiver servoing onto bare floor for its full
        # approach timeout, and it is not a substitute for one.
        if not self.grasp_is_holding():
            self.announce_failure('rack was dropped during the carry to the '
                                  'handover')
            return
        self.announce('at_handover')

        # WAIT FOR THE OTHER ROBOT BEFORE PUTTING THE RACK DOWN, not after. A
        # rack left on the floor of a corridor because the receiver never came
        # is worse than a rack still held: it is an obstacle no map knows about,
        # in the one spot both robots plan through.
        if not self.wait_for_peer('at_handover', WAIT_AT_HANDOVER):
            log.error('[relay] receiver never reached the handover -- keeping '
                      'the rack and going home with it')
            self.announce_failure('receiver never arrived')
            self.navigate_to(self.make_map_goal(*self.LAYOUT_FINAL_POSE))
            return

        if not self.set_rack_down():
            self.announce_failure('set-down failed')
            return
        self.announce('released')
        self.withdraw()
        self.announce('clear')

        # HOLD HERE UNTIL THE RECEIVER HAS THE RACK, and not because the rack
        # needs guarding - because of where home is. The carrier's park pose is
        # back in the lobby ring, which is NORTH, and the receiver is standing
        # between it and north. Setting off now sends the carrier straight at a
        # stationary robot in a corridor, which is precisely the head-on case
        # the fleet has no coordination for. Once the receiver is holding the
        # rack it leaves for the dock, northwards, and the two travel the same
        # way rather than into each other.
        if not self.wait_for_peer('holding', WAIT_AT_HANDOVER):
            log.warning('[relay] receiver never picked the rack up -- going '
                        'home anyway; the corridor may not be clear')

        log.info('=== RELAY CARRIER: parking ===')
        self.navigate_to(self.make_map_goal(*self.LAYOUT_FINAL_POSE))
        self.announce('parked')
        log.info('=== RELAY CARRIER: DONE ===')


class RelayReceiver(HandoverPeer, MissionHospitalAws):
    """r2: take the rack from the handover point and deliver it to the dock."""

    ROLE = RECEIVER
    LAYOUT_FINAL_POSE = RECEIVER_PARK

    # NO SHELF HERE, SO NO SHELF ALLOWANCE. MissionHospitalAws stops 0.12 m
    # further out than an ordinary pick because the collect bench's transfer
    # shelf protrudes that far past the rack toward the robot, and the chassis
    # rides up on it otherwise. This robot never goes near that bench: it picks
    # the rack off open floor, where the rack itself is the nearest solid thing
    # and the plain stop distance is the correct one.
    #
    # It also has to be. At the meet pose the rack's grip block reads about
    # 0.745 m ahead (0.775 to its centre, less half the 0.06 block, because the
    # camera sees the near face). Keeping the shelf allowance would put the stop
    # at 0.87 and the grasp at 0.90 - past MAX_REACH_X, i.e. clamped short and
    # closing the jaws on air.
    PAYLOAD_SHELF_OVERHANG = 0.0
    CLAW_STOP_X = OPEN_CLAW_STOP_X

    # WHERE TO LOOK FOR A RACK STANDING ON THE FLOOR.
    #
    # HOSPITAL_GATE is a camera-frame band, and the lens sits FRONT_CAM_Z =
    # 0.223 m above the floor - so its z bounds of 0.00..0.36 admit world
    # heights 0.223..0.583. That is right for the run it was written for, where
    # the rack stands on a 0.30 m shelf and its grip block is at 0.47. It is
    # wrong by more than the band is wide for a rack on the FLOOR, whose block
    # centre is at 0.17, i.e. camera z -0.053 - BELOW the gate's floor. Left
    # inherited, the receiver's approach would reject the only thing it is
    # looking for and time out on "lost the box".
    #
    # This band is the same shape, re-centred on the block where it now is:
    # world 0.023..0.323 off the floor. The x range is tightened to 1.6 m as
    # well, because unlike the bench pick this one starts from a known distance
    # (0.775 m) and there is no reason to admit red things across the room.
    HANDOVER_GATE = (0.05, 1.6, -0.5, 0.5, -0.20, 0.10)

    # Class-level default as well as the instance one below, so that
    # detect_box_front is answerable from the moment the class exists rather
    # than from the end of __init__. Nothing in the base chain detects anything
    # while constructing today, but this override is several layers below where
    # it is called from, and an AttributeError there would surface as a
    # perception failure rather than as what it is.
    _default_gate = MissionHospitalAws.HOSPITAL_GATE

    def __init__(self):
        super().__init__()
        # Which gate the inherited claw approach and grab use. It changes once,
        # for the floor pick, and goes back for the delivery - see pick_up_rack.
        # A swappable default rather than an argument because claw_approach and
        # grab_below call detect_box_front with no gate at all, several layers
        # down, and threading one through every layer for a single phase change
        # would touch code that four other missions depend on.
        self._default_gate = self.HOSPITAL_GATE

    def detect_box_front(self, timeout_sec=2.0, debug_save=False, color='blue',
                         gate=None):
        if gate is None:
            gate = self._default_gate
        return super().detect_box_front(timeout_sec, debug_save, color, gate)

    # --- picking the rack back up -------------------------------------------
    def pick_up_rack(self):
        """The ordinary claw pick, aimed at the floor instead of a shelf.

        Nothing here is new: claw_pick already takes the grasp height and the
        camera's forward correction as arguments precisely so a payload can be
        picked off any surface. The floor is just another height, and
        PAYLOAD_FLOOR_Z is already the name for it.

        The camera correction is unchanged from the bench pick and that is
        correct rather than a coincidence: it is half the 0.06 m grip block, the
        distance between the near face the camera sees and the centre the jaws
        must reach, and the block does not change size by being lower down.
        """
        log = self.get_logger()
        log.info('=== RELAY: picking the rack up off the floor ===')
        self.move_config(HOME_CONFIG, 'gripper-down ready')
        self._default_gate = self.HANDOVER_GATE
        try:
            color = self.LAYOUT_BOXES[0][0]
            return self.claw_pick(HANDOVER_POINT, color=color,
                                  grasp_z=self.PAYLOAD_FLOOR_Z,
                                  x_offset=self.LAYOUT_TABLE_X_OFFSET)
        finally:
            # Back to the shelf-height band before the delivery leg looks for
            # the dock. try/finally so a failed pick cannot leave the wrong gate
            # installed and turn one failure into two.
            self._default_gate = self.HOSPITAL_GATE

    # --- the leg ------------------------------------------------------------
    def run_mission_2(self):
        log = self.get_logger()
        log.info('=== RELAY RECEIVER: START ===')
        if not self.wait_for_localization():
            self.announce_failure('never localized')
            return
        self.announce('start')

        # DO NOT SET OFF UNTIL THE CARRIER ACTUALLY HAS THE RACK. Leaving on
        # start would put both robots in the south corridor at once, driving
        # opposite ways, for the whole length of the collect leg. Waiting costs
        # nothing - the receiver has nothing to do until there is something to
        # receive - and by the time this releases, the carrier is at the bench
        # 18 m away and the corridor between here and the meet pose is empty.
        if not self.wait_for_peer('collected', WAIT_FOR_COLLECT):
            self.announce_failure('carrier never collected the rack')
            return

        log.info('=== RELAY RECEIVER: driving to the handover ===')
        self._set_yaw_goal_tolerance(TIGHT_YAW_TOLERANCE)
        nav_ok = self.navigate_to(self.make_map_goal(*HANDOVER_RECEIVER))
        self._set_yaw_goal_tolerance(DEFAULT_YAW_TOLERANCE)
        if not nav_ok:
            self.announce_failure('could not reach the handover pose')
            return
        self.announce('at_handover')

        # WAIT FOR `clear`, NOT FOR `released`. The rack is on the floor at
        # `released`, but the carrier is still standing over it with an arm
        # extended, 1.55 m away and well inside this robot's local costmap. The
        # pick starts with a visual servo that may drive the base forward, and
        # driving it toward a robot that has not finished retracting is how two
        # machines that avoid each other only by LIDAR end up touching.
        if not self.wait_for_peer('clear', WAIT_AT_HANDOVER):
            self.announce_failure('carrier never handed the rack over')
            return

        if not self.pick_up_rack():
            self.announce_failure('could not pick the rack up off the floor')
            return
        self.announce('holding')

        log.info('=== RELAY RECEIVER: delivering to the dock ===')
        # The dock, its height and its id, read off the inherited layout rather
        # than re-imported: this is the same single-entry LAYOUT_COLUMNS the
        # single-robot run delivers to, and the delivery leg is that run's.
        color = self.LAYOUT_BOXES[0][0]
        tag_id, height, col_xy = self.LAYOUT_COLUMNS[0]
        if not self.deliver_to_column(tag_id, height, col_xy, color):
            self.announce_failure('delivery failed')
            return
        self.announce('delivered')

        log.info('=== RELAY RECEIVER: parking ===')
        self.navigate_to(self.make_map_goal(*self.LAYOUT_FINAL_POSE))
        self.announce('parked')
        log.info('=== RELAY RECEIVER: DONE ===')


def carrier():
    _run(RelayCarrier)


def receiver():
    _run(RelayReceiver)
