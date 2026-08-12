#!/usr/bin/env python3
"""Blood sample transport: collect a rack in the lab, deliver it to the ward.

This is the colour-sorting mission relocated, not a rewrite. Mission2's
run_mission_2 is layout-agnostic - it reads only LAYOUT_* attributes and never
the warehouse module globals - so one pickup and one drop-off, with the right
coordinates, is a single-item run of exactly the same pick / carry / place /
verify sequence.

What genuinely differs from the warehouse, and why:

  * The payload is a rack, not a cube. It is grasped by a red block on a gantry
    0.17 m above its own tray, so the fingertip frame sits 0.17 m above whatever
    surface the rack must stand on - not half a cube. That is PAYLOAD_GRIP_HEIGHT,
    and getting it wrong drives the rack through the shelf rather than onto it.

  * The model is sample_rack, not box_red, so the DetachableJoint topics differ
    even though the detection colour is still red. That is GRASP_MODELS.

  * The drop target is a green dock, while the payload is red. In the warehouse
    each column is painted its box's colour, so the post-release check looks for
    that one colour; here it must look for the rack, not the dock, which is what
    PLACE_VERIFY_COLOR is for.

  * The carry is long. Lab to ward is ~20 m through a corridor with pedestrians
    and trolleys in it, against a few metres across an open room. run_mission_2
    already checks grasp_is_holding() on arrival before placing, and that check
    matters far more here.

COORDINATES. Everything below is in the MAP frame, which for hospital_lab.sdf is
the world frame shifted by -SPAWN, i.e. map = world + (6.5, 0) - the robot spawns
at world (-6.5, 0) and slam_toolbox put the map origin there. World coordinates
are given alongside each constant so they can be checked against the world file
directly.

HEIGHTS, all above the floor:
    lab bench transfer shelf top   0.30    (lab_bench, shelf slab)
    rack standing on it            0.30    tray underside
    its red grip block centre      0.47    = 0.30 + 0.17
    ward bench shelf top           0.30
    delivery dock top              0.38    = 0.30 + 0.08 dock
    rack once delivered            0.38    tray underside
    its grip block once delivered  0.55    = 0.38 + 0.17
"""
import math

from pickplace_arm_bringup.mission_2 import Mission2, _run
from pickplace_arm_bringup.pick_and_place import GROUND_Z

# --- map frame = world + (6.5, 0) --------------------------------------------
MAP_DX = 6.5


def w2m(x, y):
    """World coordinates to map coordinates."""
    return x + MAP_DX, y


# --- pickup: lab bench, north wall of the lab, shelf facing south -------------
# The bench stands at world (-5.0, 3.4) rotated a quarter turn, so its transfer
# shelf spans world y 2.72..3.02 with its top at z = 0.30. The rack is spawned
# in the middle of that shelf.
RACK_WORLD = (-5.0, 2.87)
RACK_MAP = w2m(*RACK_WORLD)
# The robot squares up 1.00 m south of the rack facing +y (north), matching the
# warehouse's 1.00 m table standoff: claw_approach servos the last ~0.30 m in on
# the front camera, and stopping closer than this puts the Husky's bumper
# (base_link x = 0.4937) into the shelf. At 1.00 m the bumper sits 0.36 m clear
# of the shelf's near edge, essentially the warehouse's own 0.38 m.
PICKUP_APPROACH = (RACK_MAP[0], RACK_MAP[1] - 1.00, math.pi / 2)
# Fingertip z to grasp the rack by its grip block: shelf top + grip height.
RACK_GRIP_HEIGHT = 0.170
PICKUP_GRASP_Z = GROUND_Z + 0.30 + RACK_GRIP_HEIGHT

# --- delivery: ward bench, shelf facing west toward the door -----------------
# Bench at world (14.5, 0.0) unrotated, shelf spanning world x 13.85..14.15 with
# its top at 0.30; the dock sits in the middle of it and its own top is 0.38.
DOCK_WORLD = (14.0, 0.0)
DOCK_MAP = w2m(*DOCK_WORLD)
DOCK_TOP = 0.38

# --- parking -----------------------------------------------------------------
# Back at the spawn point in the lab, out of the corridor and clear of the
# bench, so a repeat run starts from where the first one did.
PARK_WORLD = (-6.5, 0.0)
PARK_MAP = w2m(*PARK_WORLD)


class MissionHospital(Mission2):
    """Lab to ward sample transport in hospital_lab.sdf."""

    # One pickup, one delivery. run_mission_2 zips these together, so a
    # single-entry pair is a single-item run of the same sequence.
    LAYOUT_BOXES = [('red', RACK_MAP)]
    LAYOUT_COLUMNS = [(0, DOCK_TOP, DOCK_MAP)]
    LAYOUT_TABLE_APPROACH = PICKUP_APPROACH
    LAYOUT_TABLE_GRASP_Z = PICKUP_GRASP_Z
    LAYOUT_FINAL_POSE = (PARK_MAP[0], PARK_MAP[1], 0.0)

    # The rack, like the boxes, is detected by its near face while the jaws have
    # to reach its centre; the grip block is the same 0.06 m across, so the same
    # half-block correction applies and the inherited value is already right.
    # Named explicitly rather than inherited silently because it is geometry of
    # THIS payload that happens to coincide.
    LAYOUT_TABLE_X_OFFSET = 0.030

    # --- payload geometry ----------------------------------------------------
    GRASP_MODELS = {'red': 'sample_rack'}
    PAYLOAD_GRIP_HEIGHT = RACK_GRIP_HEIGHT
    # Where the grip block reads if the rack ends up on the FLOOR rather than
    # the dock - the whole point of the post-release check.
    PAYLOAD_FLOOR_Z = GROUND_Z + RACK_GRIP_HEIGHT
    # The drop target is the green dock, not another red thing.
    TARGET_COLOR = 'green'
    # Look for the RACK after release, not the dock it was placed on.
    PLACE_VERIFY_COLOR = 'red'
    # Approach the dock from the WEST (-x), i.e. from the corridor the robot
    # arrives through. The bench body sits east of its own shelf, so the
    # warehouse's +x default parks the base behind the bench with the dock out
    # of sight on the far side - measured on the first run, which drove to world
    # x 15.43 and stared at the back of the bench.
    PLACE_APPROACH_DIR = (-1.0, 0.0)

    # --- detection gates -----------------------------------------------------
    # Camera-frame (X-forward, Y-left, Z-up, metres), relative to the lens at
    # FRONT_CAM_Z = 0.223 m. A hospital ward is full of incidental colour, so
    # both detections are gated the same way the Tugbot warehouse's are.
    #
    # What has to stay inside it:
    #   rack grip block on the lab shelf, centre 0.47   -> cam z +0.25
    #   delivery dock, spanning 0.30..0.38              -> cam z +0.08..+0.16
    # so the band runs from just below the dock to just above the grip block.
    HOSPITAL_GATE = (0.05, 2.5, -0.7, 0.7, 0.0, 0.36)
    COLUMN_DETECT_GATE = HOSPITAL_GATE

    def detect_box_front(self, timeout_sec=2.0, debug_save=False, color='blue',
                         gate=None):
        if gate is None:
            gate = self.HOSPITAL_GATE
        return super().detect_box_front(timeout_sec, debug_save, color, gate)


def main():
    _run(MissionHospital)


if __name__ == '__main__':
    main()
