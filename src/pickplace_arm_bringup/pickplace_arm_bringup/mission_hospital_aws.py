#!/usr/bin/env python3
"""Blood sample transport in the AWS hospital: south wing -> lobby.

The same Mission2 sequence as mission_hospital.py, relocated again. That class
is layout-agnostic - run_mission_2 reads only LAYOUT_* attributes - so a new
building is coordinates and payload geometry, not a second implementation. Most
of the payload geometry is identical to the hospital_lab run because it is the
same rack on the same benches, so this file is mostly the layout.

WHY THIS IS THE SINGLE-ROBOT VERSION OF A TWO-ROBOT PLAN. The goal is a relay:
one robot collects, hands the rack to a second robot half way, and the second
delivers. This runs the whole route with ONE robot first, because every part of
it except the handover is shared with the relay - the same collect pose, the
same grasp, the same delivery. Get this working and the relay is that route cut
in half with a handover spliced into the join.

THE MAP FRAME IS THE WORLD FRAME HERE, so there is no w2m() and no MAP_DX. The
map was not built by driving the robot around: aws_hospital_map.py slices the
world's own collision geometry at LIDAR height, in world coordinates (see
maps/aws_hospital.yaml, origin [-13.50, -36.00]). AMCL confirms the alignment -
both robots converge on their exact spawn poses with no correction at all. In
hospital_lab the map came from slam_toolbox starting at the robot's spawn point,
which is why that file has to shift every coordinate by +6.5 in x.

That map includes the FURNITURE, not just the walls, and this route is why. The
first version carried walls only, so the nurses' station - which sits squarely
between the lobby and the southern corridor - was invisible to the global
planner. It routed straight through the desk and the robot drove into it.

HEIGHTS, all above the floor, unchanged from the hospital_lab run:
    bench transfer shelf top        0.30
    rack standing on it             0.30    tray underside
    its red grip block centre       0.47    = 0.30 + 0.17
    delivery dock top               0.38    = 0.30 + 0.08 dock
    rack once delivered             0.38
    its grip block once delivered   0.55    = 0.38 + 0.17
"""
import math

from pickplace_arm_bringup.mission_2 import Mission2, _run
from pickplace_arm_bringup.pick_and_place import GROUND_Z

from pickplace_arm_bringup.hospital_aws_layout import (  # noqa: E402
    BENCH_COLLECT, BENCH_DELIVER, DOCK, DOCK_TOP, HANDOVER_CARRIER,
    HANDOVER_RECEIVER, RACK,
)

# The bench faces east, so the robot squares up 1.00 m EAST of the rack looking
# west at it. Same 1.00 m standoff the warehouse and hospital_lab use:
# claw_approach servos the last ~0.30 m in on the front camera, and closer than
# this puts the Husky's bumper (base_link x = 0.4937) into the shelf.
PICKUP_APPROACH = (RACK[0] + 1.00, RACK[1], math.pi)
RACK_GRIP_HEIGHT = 0.170
PICKUP_GRASP_Z = GROUND_Z + 0.30 + RACK_GRIP_HEIGHT

# --- parking ------------------------------------------------------------------
# r1's pose in the fleet ring. A repeat run starts where the first one did.
PARK = (-2.25, 11.50, 0.0)


class MissionHospitalAws(Mission2):
    """South wing to lobby sample transport in aws_hospital.sdf."""

    LAYOUT_BOXES = [('red', RACK)]
    LAYOUT_COLUMNS = [(0, DOCK_TOP, DOCK)]
    LAYOUT_TABLE_APPROACH = PICKUP_APPROACH
    LAYOUT_TABLE_GRASP_Z = PICKUP_GRASP_Z
    LAYOUT_FINAL_POSE = PARK

    # The collect run is a long route - the direct line south is blocked by the
    # nurses' station and the lift core, so Nav2 goes round the west corridor.
    # Measured against the map, not guessed, and the default 120 s aborted a
    # drive that was progressing perfectly normally.
    NAV_TIMEOUT_SEC = 300.0

    # Leave the bench under Nav2 rather than reversing blind. The collect bench
    # stands in a bay with a 0.30 m transfer shelf sticking out toward the
    # robot, and a blind reverse along a heading the visual servo has already
    # swung 55 degrees put the chassis up on that shelf twice.
    BACK_OFF_VIA_NAV = True

    # Stop further out than a box pick would, because lab_bench's transfer shelf
    # sticks out past the rack standing on it: the shelf slab runs from local
    # x -0.35 to -0.65 and the rack sits at -0.53, so 0.12 m of it is between
    # the robot and the payload. Measured off models/lab_bench/model.sdf, not
    # estimated. Stopping 0.75 m from the rack therefore leaves the bumper
    # (base_link x = 0.494) only 0.136 m off a 0.30 m shelf edge, against a
    # 0.165 m wheel, and the chassis rides up onto it - measured twice.
    #
    # BUT 0.75 + 0.12 = 0.87 IS TOO FAR TO GRASP FROM, AND THAT WENT UNNOTICED
    # BECAUSE THE FAILURE IS SILENT. The camera reads the grip block's near
    # FACE, and the jaws have to reach its CENTRE, half a block (0.030) further:
    #
    #     stop 0.87  ->  grasp wants 0.900  ->  MAX_REACH_X caps it at 0.850
    #
    # so every pick closed 0.05 m short of centre on a 0.06 m block - i.e. on
    # its front edge. Three runs out of three did exactly this, and all three
    # reported a good grasp, because the finger-gap check only asks whether
    # something is between the jaws, not where. The visible symptom was the rack
    # riding at 14 degrees of pitch (ground truth rpy 0.008, 0.2455, 0.296),
    # which is what being carried by one edge looks like.
    #
    # The old note here claimed "the arm still reaches: the grasp point ends up
    # 0.795 m from fr3_link0". That arithmetic was done against the stop
    # distance rather than the grasp point: 0.900 in base_link is 0.820 from
    # fr3_link0 at x = 0.08, which is 96% of the FR3's 0.855 m envelope -
    # near-singular, and past the x = 0.85 that /compute_ik was actually swept
    # to. Raising MAX_REACH_X to cover it would be reaching into that corner.
    #
    # So the base stops closer instead, by exactly enough that the grasp lands
    # inside the verified envelope with headroom rather than on the cap:
    #
    #     stop 0.80  ->  grasp 0.830        (0.02 m below MAX_REACH_X)
    #     shelf edge at 0.80 - 0.12 = 0.68  (bumper clears it by 0.186 m)
    #
    # That is 0.05 m less bumper clearance than 0.87 gave and 0.05 m MORE than
    # the 0.136 m that actually failed - and unlike 0.87 it grips the block
    # through its middle. If the chassis ever touches the shelf again, the lever
    # to pull is this number, and the constraint it must respect is
    # CLAW_STOP_X + LAYOUT_TABLE_X_OFFSET <= MAX_REACH_X.
    PAYLOAD_SHELF_OVERHANG = 0.12
    CLAW_STOP_X = 0.80

    # The rack is detected by its near face while the jaws reach its centre; the
    # grip block is the same 0.06 m cube the boxes present, so the inherited
    # half-block correction is already right. Named rather than inherited
    # silently because it is geometry of THIS payload that happens to coincide.
    LAYOUT_TABLE_X_OFFSET = 0.030

    # --- payload geometry ----------------------------------------------------
    GRASP_MODELS = {'red': 'sample_rack'}
    PAYLOAD_GRIP_HEIGHT = RACK_GRIP_HEIGHT
    PAYLOAD_FLOOR_Z = GROUND_Z + RACK_GRIP_HEIGHT
    TARGET_COLOR = 'green'
    PLACE_VERIFY_COLOR = 'red'
    # Approach the dock from the NORTH (+y). The delivery bench now stands
    # against the information desk at yaw -pi/2, which turns its shelf to face
    # north into the lobby; the desk itself is immediately south of the bench,
    # so the old (0, -1) approach would drive the robot at the counter and leave
    # the dock out of sight on the far side of the carcass.
    PLACE_APPROACH_DIR = (0.0, 1.0)

    # --- detection gates -----------------------------------------------------
    # Camera-frame (X-forward, Y-left, Z-up), relative to the lens at
    # FRONT_CAM_Z = 0.223 m. Same band as the hospital_lab run, and needed at
    # least as much: this lobby has coloured chairs, a vending machine and
    # walking people in it.
    #
    #   rack grip block on the shelf, centre 0.47  -> cam z +0.25
    #   delivery dock, spanning 0.30..0.38         -> cam z +0.08..+0.16
    HOSPITAL_GATE = (0.05, 2.5, -0.7, 0.7, 0.0, 0.36)
    COLUMN_DETECT_GATE = HOSPITAL_GATE

    def detect_box_front(self, timeout_sec=2.0, debug_save=False, color='blue',
                         gate=None):
        if gate is None:
            gate = self.HOSPITAL_GATE
        return super().detect_box_front(timeout_sec, debug_save, color, gate)


def main():
    _run(MissionHospitalAws)


if __name__ == '__main__':
    main()
