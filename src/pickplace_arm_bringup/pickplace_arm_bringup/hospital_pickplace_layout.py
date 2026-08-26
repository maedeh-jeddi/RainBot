"""Where the pick-and-place run stands in aws_hospital.sdf.

THIS IS THE WAREHOUSE MISSION'S LAYOUT, MOVED, NOT A NEW ONE. Every offset
below is lifted verbatim from mission_pickPlace.launch.py / mission_2.py -- the
table 2.30 m ahead of the spawn, the columns 1.0 m behind it, the racks 0.22 m
apart on the table, the parking spot 1.8 m to the right. What changes is only
where that whole rigid arrangement is dropped into the building, and which way
it faces. Keeping it rigid is the point: the tuning behind those offsets
(CLAW_STOP_X, NAV_STANDOFF, COLUMN_STOP_X, the yaw tolerances) was measured
against this exact geometry, and none of it has to be re-earned.

ONE SOURCE OF TRUTH, IMPORTED BY TWO THINGS: the launch that spawns the props
and the mission that drives to them. They fell out of sync exactly once in this
project's history and it cost a debugging session -- the launch spawned a bench
the map knew nothing about, Nav2's global planner routed the robot straight
through it, and the robot ended up wedged against a bench that from the
planner's point of view was thin air.

DELIBERATELY FREE OF ROS IMPORTS, so the map generator and any offline check
can read it without dragging in pymoveit2 and rclpy just to learn where a
column stands.

Poses are (x, y) or (x, y, yaw) in the world frame, which is also the map frame
here: maps/aws_hospital.yaml is generated from the world's own geometry by
aws_hospital_map.py, so map == world exactly, with no SLAM drift in between.
"""
import math

# --- where the whole arrangement sits -----------------------------------------
#
# ANCHOR is the robot's spawn pose, and the layout's origin: the table is
# straight ahead of it, the columns straight behind. YAW is which way "ahead"
# points in the world.
#
# CHOSEN BY MEASUREMENT, NOT BY EYE. Every (x, y, yaw) on a 0.25 m grid over the
# whole building, at 15 degree steps, was scored against maps/aws_hospital.pgm
# for: all six robot poses and the straight runs between them clearing the
# Husky's 0.598 m circumscribed radius, all six props standing on free floor
# with 0.30 m of their own, and the whole thing staying out of the fleet ring
# (the four settled robot poses in v1-devel-clone's hospital_aws_layout, so that
# a later step can park four Huskys there without moving any of this). Of 8423
# candidates this was the best by a clear margin.
#
# THEN RE-MEASURED AGAINST THE COLLISION MESHES AT EVERY HEIGHT, because the map
# is a single horizontal slice at the LIDAR's height -- 0.4466 m when this was
# written, 0.3143 m since the scanner was hung under the plate -- and the table
# (0.30 m top) and two of the three columns live entirely underneath either of
# them. Slicing at every height is what makes this measurement independent of
# where the sensor sits, which is why it did not have to be redone. Slicing the
# world's 204,863 collision triangles at 5 cm steps from 0.05 to 1.10 m gives,
# centre of item to nearest solid thing:
#
#   robot spawn     2.62 m        table            2.53 m
#   table approach  2.45 m        column 0 (0.30)  2.02 m
#   col approaches  2.20-2.88 m   column 1 (0.40)  2.30 m
#   park pose       2.14 m        column 2 (0.50)  2.17 m
#
# i.e. 1.4 m of daylight past the circumscribed radius at the worst pose. That
# is the check that matters here: AWS prop origins can sit metres from their own
# geometry, which is how an earlier attempt in this project parked a robot
# inside a reception desk.
#
# THE ROOM IS THE WEST SIDE OF THE LOBBY, north-west of the nurses' station --
# within a metre of the spot the standalone hospital bring-up independently
# picked as "the best-balanced clear spot in the building".
ANCHOR = (-4.00, 8.00)
YAW = math.pi / 2.0          # the run faces north, up the lobby


def _place(x, y):
    """A layout-frame point in world coordinates."""
    c, s = math.cos(YAW), math.sin(YAW)
    return (ANCHOR[0] + c * x - s * y, ANCHOR[1] + s * x + c * y)


def _pose(x, y, yaw=0.0):
    """A layout-frame pose in world coordinates."""
    wx, wy = _place(x, y)
    return (wx, wy, yaw + YAW)


# --- the props ----------------------------------------------------------------
#
# Spawn z is the MODEL ORIGIN, not the visible bottom, and the three models
# disagree about where that is -- which is exactly the mistake worth spelling
# out. The table box is centred, so it spawns at half its height. A rack carries
# its origin on its own tray underside, so it spawns at the height of the
# surface it stands on. The columns are modelled from their base, so they spawn
# at 0.
TABLE_TOP = 0.30
TABLE = _pose(2.30, 0.0)
TABLE_Z = TABLE_TOP / 2.0

# Grip block centre above the rack's own tray underside; see models/rack_red.
# The single number that the rack costs over a box, and the one every height in
# the mission is re-derived from.
RACK_GRIP_HEIGHT = 0.170

# (colour, world xy) in pick order, matching the columns below one for one.
# The y offsets are the boxes' 0.22 m spacing unchanged: the racks are 0.12 m
# wide, so that leaves 0.10 m between neighbours and 0.16 m between the grip
# blocks the jaws close on.
RACKS = [
    ('red',   _place(2.30, -0.22)),
    ('green', _place(2.30,  0.00)),
    ('blue',  _place(2.30,  0.22)),
]
RACK_SPAWN_Z = TABLE_TOP

# (column id, height above the floor, world xy). Heights 30/40/50 cm and the
# 0.45 m spacing are the warehouse's, unchanged.
COLUMNS = [
    (0, 0.30, _place(-1.0, -0.45)),   # red
    (1, 0.40, _place(-1.0,  0.00)),   # green
    (2, 0.50, _place(-1.0,  0.45)),   # blue
]
# Columns face back down the layout's -x, i.e. towards the robot arriving from
# the table side, exactly as they do in the warehouse (yaw pi there).
COLUMN_YAW = math.pi + YAW

# --- the poses the robot drives to --------------------------------------------
SPAWN = (ANCHOR[0], ANCHOR[1], YAW)
TABLE_APPROACH = _pose(1.30, 0.0)
FINAL_POSE = _pose(0.0, -1.8)
# Which side of a column the base is sent to for the placement standoff, as a
# unit vector in the WORLD frame (see Mission2.PLACE_APPROACH_DIR). In the
# layout frame it is always +x -- the robot comes from the table, which is on
# that side -- so here it is that direction rotated into the world.
PLACE_APPROACH_DIR = (math.cos(YAW), math.sin(YAW))
