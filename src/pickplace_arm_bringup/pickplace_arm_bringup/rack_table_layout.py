"""Where the four rack tables stand in aws_hospital.sdf, and where the racks sit
on them.

THREE COLLECTION POINTS, ONE DELIVERY POINT. Each collection table carries one
sample rack; the delivery table has three slots in a row and ends up carrying
all three. That is the shape of the job: three robots, three errands, one
destination.

ONE SOURCE OF TRUTH, IMPORTED BY EVERYTHING THAT NEEDS IT -- the launch that
spawns the props, the map generator that has to stamp the tables, and later the
task manager that dispatches robots to them. They fell out of sync exactly once
in this project's history and it cost a debugging session: the launch spawned a
bench the map knew nothing about, the global planner routed straight through it,
and the robot wedged itself against a prop that as far as Nav2 was concerned was
thin air.

DELIBERATELY FREE OF ROS IMPORTS, so the map generator and any offline clearance
check can read it without dragging in rclpy and pymoveit2.

Poses are (x, y) or (x, y, yaw) in the world frame, which is also the map frame:
maps/aws_hospital.yaml is generated from the world's own geometry, so map ==
world exactly.


FRAME CONVENTION, because every offset below depends on it
----------------------------------------------------------
A table's own frame has its LONG axis (1.327 m) along local x and its SHORT axis
(0.668 m) along local y, with the origin centred on the footprint and the feet
at z = 0. The robot always works from the table's local -y side. So:

    local +x   along the table, left to right as the robot sees it
    local -y   towards the robot
    table yaw  rotates that frame into the world

and a robot standing at the table faces the table's local +y direction, i.e.
yaw + pi/2 in world terms. Getting that sign wrong points the robot away from
the table while every clearance check still passes, which is exactly what
happened the first time.
"""
import math

# --- the table model ----------------------------------------------------------
#
# Measured off models/rack_table's collision mesh rather than taken from the
# model card: 1.327 x 0.668 m with a flat top at 0.3238, origin centred.
TABLE_LONG = 1.327
TABLE_SHORT = 0.668
TABLE_TOP = 0.3238
# Spawn z is the model origin, and this model's origin is at its FEET, so it
# spawns at 0 and its top lands at TABLE_TOP.
TABLE_SPAWN_Z = 0.0

# --- where a rack stands on a table -------------------------------------------
#
# THE BUMPER DECIDES THIS NUMBER, not the middle of the table.
#
# The base stops with the payload 0.78 m ahead of base_link (CLAW_STOP_X 0.75
# plus TABLE_X_OFFSET 0.03) and the Husky's front bumper is at 0.4937. So the
# table's near edge has to sit beyond 0.4937 or the chassis is inside the table
# before the jaws reach the rack.
#
# A rack centred on this table would be TABLE_SHORT/2 = 0.334 m from the near
# edge, putting that edge at 0.78 - 0.334 = 0.446 -- i.e. 0.048 m INSIDE the
# bumper. The rack therefore sits well forward of centre:
#
#     rack 0.15 m in from the near edge  ->  edge at 0.63 in base_link
#                                        ->  0.136 m of bumper clearance
#                                        ->  0.07 m of table lip in front of the
#                                            rack's own 0.08 m half-depth
#
# For comparison the 0.25 m-deep box table this replaces left 0.16 m of bumper
# clearance, so this is the same order of margin, reached a different way.
RACK_EDGE_INSET = 0.15
RACK_LOCAL_Y = -(TABLE_SHORT / 2.0 - RACK_EDGE_INSET)      # -0.184
# Racks stand ON the top, and the rack model carries its origin on its own tray
# underside, so this is simply the table top's height.
RACK_SPAWN_Z = TABLE_TOP
# Height of the rack's grip block CENTRE above its own tray underside -- the one
# number a rack costs over a box, and the one every placement height is derived
# from. Defined here, next to the table it stands on, because the two are always
# used together: a rack on this table puts its grasp point at
# TABLE_TOP + RACK_GRIP_HEIGHT above the floor. See models/rack_red.
RACK_GRIP_HEIGHT = 0.170

# --- where the robot stands ---------------------------------------------------
#
# Nav2 parks the base with the rack about 1.30 m ahead -- the same standoff the
# single-robot run uses, and for the same reason: it is a comfortable detection
# range for the front camera, from which claw_approach servos the last stretch
# in. It is NOT the grasp distance; the visual servo decides that.
NAV_STANDOFF = 1.30
STANDOFF_LOCAL_Y = RACK_LOCAL_Y - NAV_STANDOFF             # -1.484

# THE DELIVERY TABLE IS APPROACHED CLOSER, because nothing has to be SEEN there.
#
# 1.30 m is a detection distance: at a collection table the front camera has to
# find the rack from the standoff, and claw_approach then servos the last
# stretch in. Placing is the other way round -- the robot already holds the
# rack, and the only thing that matters is that the slot ends up inside the
# arm's 0.85 m reach.
#
# Parking at 1.30 m left a 0.48 m gap for _creep_forward to close, and that
# creep measured 0.000 m of movement: the mission node and Nav2's
# controller_server BOTH publish to the same cmd_vel topic (publisher count 2),
# so a creep issued just after a goal completes is arguing with whatever the
# controller last said. Closing most of the gap with the nav goal itself means
# there is far less left to creep, and the creep that remains is short.
#
# 0.85 m keeps the geometry safe: the slot sits 0.15 m in from the table's near
# edge, so that edge lands 0.70 m ahead of base_link against a bumper at 0.494 --
# 0.21 m of clearance, comparable to what the collection tables leave. It also
# puts the slot inside the arm's reach AT THIS HEIGHT on arrival, so the common
# case needs no creep at all; see DELIVERY_MAX_REACH_X in mission_delivery.
DELIVERY_NAV_STANDOFF = 0.85
DELIVERY_STANDOFF_LOCAL_Y = RACK_LOCAL_Y - DELIVERY_NAV_STANDOFF


def _to_world(table, lx, ly):
    """A point in a table's local frame, in world coordinates."""
    tx, ty, tyaw = table
    c, s = math.cos(tyaw), math.sin(tyaw)
    return (tx + c * lx - s * ly, ty + s * lx + c * ly)


def _robot_yaw(table):
    """Heading of a robot working at this table: it faces the table's local +y."""
    return table[2] + math.pi / 2.0


# --- the collection points ----------------------------------------------------
#
# CHOSEN BY SEARCH AND VERIFIED IN 3D. Every (x, y, yaw) on a 0.5 m grid at 15
# degree steps was scored, keeping only poses where the table's own footprint
# stands clear, the robot's standoff pose clears its 0.598 m circumscribed
# radius, the 2.0 x 1.4 m strip the robot REVERSES OUT THROUGH is clear too, and
# that standoff is connected to the fleet's own floor on a map eroded by the
# robot radius -- i.e. a Husky can genuinely drive there.
#
# THE MAP CANNOT BE THE ONLY CHECK, AND THAT IS NOT A DETAIL HERE. The map is
# one horizontal slice at the LIDAR's 0.4466 m, and these tables are 0.32 m
# tall -- entirely underneath it. The first two placements this search produced
# passed the map easily and turned out to have 0.00 m and 0.10 m of real
# clearance, because something low sat exactly there. Every pose below is scored
# against the collision meshes sliced every 5 cm from 0.05 m to 1.15 m instead.
#
# Measured for the four finally chosen, table footprint / robot standoff /
# back-out strip:
#
#   delivery    2.25 m   2.43 m   1.40 m
#   collect_0   0.72 m   1.60 m   0.90 m
#   collect_1   0.84 m   1.65 m   0.95 m
#   collect_2   0.60 m   1.57 m   0.85 m
#
# THREE DIFFERENT WINGS, ON PURPOSE, so the three errands are genuinely
# different lengths -- which is what makes assigning them to robots a decision
# rather than a formality.
#
# COLLECT_2 IS NOT WHERE THE SEARCH FIRST PUT IT, AND THE REASON IS THE WHOLE
# LESSON OF THIS BLOCK. Its first home was (-8.50, -22.50), which passed every
# check above -- table fits, standoff fits, back-out strip fits, standoff
# connected to reception on floor eroded by the robot's 0.598 m circumscribed
# radius. A robot sent there failed twice from a clean pose, ending up wedged
# against the table's south corner both times.
#
# The checks were all true and all beside the point. That table stood beside the
# ONLY doorway into its room, and the standoff was on the far side of it, so the
# robot had to squeeze past the table to reach its own destination -- through a
# gap the table itself narrowed to 1.25 m against a robot that needs 1.196 m to
# turn. Connectivity after eroding by the circumscribed radius says a route
# exists with ZERO margin; it does not say Nav2, checking the true 0.99 x 0.67
# rectangle from a pose estimate with its own error, can drive it.
#
# So the rule that produced the pose below is: THE STANDOFF MUST BE ON THE SIDE
# THE ROBOT ARRIVES FROM, in a space wide enough that getting to it is not
# itself a manoeuvre. (-7.50, -26.50) stands in the middle of the southern hall
# with several metres clear on every side and the approach along that same hall,
# so there is no doorway in the problem at all.
#
# (name, colour of the rack it carries, table x, y, yaw)
COLLECTION_TABLES = [
    ('collect_0', 'red',   (-8.50,  -5.00, math.pi)),      # west wing, north end
    ('collect_1', 'green', (8.50, -19.50, 0.0)),           # east wing, south
    ('collect_2', 'blue',  (-7.50, -26.50, 3.0 * math.pi / 2)),   # far south hall
]

# --- the delivery point -------------------------------------------------------
#
# In the lobby, 3.7 m from the fleet's own formation and clear of every robot in
# it by more than 2.8 m, so the three can come and go without the table being in
# the middle of their standing positions. It is also the shortest hop of the
# four -- 3.8 m of driveable route from reception -- which is what makes it a
# plausible front-desk drop-off rather than an arbitrary corner.
DELIVERY_TABLE = ('delivery', (-3.50, 10.00, math.pi))

# THREE SLOTS IN A ROW along the table's long axis, which is what "the first
# available position in a row" needs to mean geometrically.
#
# 0.30 m apart: a rack is 0.16 m along this axis, so that is 0.14 m of air
# between neighbours -- enough that lowering one between two others does not
# graze them, and enough that the front camera sees a single rack rather than a
# smear of three. Three slots span 0.60 m inside a 1.327 m top, so the outer two
# still have 0.30 m of table beyond them.
DELIVERY_SLOT_SPACING = 0.30
DELIVERY_SLOT_LOCAL_X = (-DELIVERY_SLOT_SPACING, 0.0, DELIVERY_SLOT_SPACING)


# --- derived poses, which is all anything outside this module should use -------
def collection_points():
    """[(name, colour, table_pose, rack_xy, robot_pose)] for the three pickups."""
    out = []
    for name, colour, table in COLLECTION_TABLES:
        rack = _to_world(table, 0.0, RACK_LOCAL_Y)
        stand = _to_world(table, 0.0, STANDOFF_LOCAL_Y)
        out.append((name, colour, table, rack, stand + (_robot_yaw(table),)))
    return out


def delivery_slots():
    """[(index, slot_xy, robot_pose)] for the three places a rack can be put
    down, left to right as the robot sees them.

    THE PER-SLOT ROBOT POSE IS GEOMETRY, NOT A DRIVING INSTRUCTION. Measured on
    a real run, a robot sent to each of these three standoffs in turn arrived
    0.34, 0.47 and 0.45 m from the pose it was given -- every goal succeeded,
    because that is inside Nav2's own 0.20 m tolerance plus AMCL's error, but it
    is LARGER THAN THE 0.30 m SLOT SPACING. Driving to a slot's standoff
    therefore cannot address that slot: the robot may well end up squarely in
    front of its neighbour.

    So whatever fills these slots should park ONCE, at DELIVERY_STANDOFF below,
    and reach the 0.30 m sideways with the ARM, which is precise. The mission
    code already commands a lateral y for placement, so this is the arm doing
    what it already does rather than anything new.
    """
    _, table = DELIVERY_TABLE
    out = []
    for i, lx in enumerate(DELIVERY_SLOT_LOCAL_X):
        slot = _to_world(table, lx, RACK_LOCAL_Y)
        stand = _to_world(table, lx, STANDOFF_LOCAL_Y)
        out.append((i, slot, stand + (_robot_yaw(table),)))
    return out


def delivery_standoff():
    """The ONE pose a robot drives to in order to use the delivery table.

    Centred on the table, so all three slots sit within +/- 0.30 m of the
    gripper's lateral reach from here. See delivery_slots() for why there is a
    single standoff rather than one per slot.
    """
    _, table = DELIVERY_TABLE
    return _to_world(table, 0.0, DELIVERY_STANDOFF_LOCAL_Y) + (_robot_yaw(table),)


def delivery_table_pose():
    return DELIVERY_TABLE[1]


# --- what belongs in the map --------------------------------------------------
#
# ALL FOUR TABLES, and this is a genuine trade-off rather than an obvious call.
#
# AGAINST: a table top at 0.3238 m is BELOW the LIDAR's 0.4466 m scan plane, so
# the sensor can never return a point on one. Stamping geometry the scan cannot
# see puts occupied cells in the map that no measurement will ever support,
# which is the situation aws_hospital_map.py's own docstring warns about.
#
# FOR, and decisively: the costmaps have no other obstacle source. Their only
# observation source is that same LIDAR (see nav2_params.yaml), so a table left
# out of the map is invisible to the global planner AND to the local costmap,
# and the robot drives into it. The delivery table sits in the lobby, which is
# the hub every robot crosses to reach anywhere else -- it would be hit.
#
# This project has already paid for that lesson twice, with the nurses' station
# and with a delivery bench, both of which ended with a robot wedged against a
# prop the planner did not know about.
#
# The AMCL cost is small and bounded: likelihood_field scores each scan ENDPOINT
# against the nearest occupied cell, so extra occupied cells can only pull a
# real endpoint's nearest-neighbour distance down slightly. It cannot invent
# returns. Four tables totalling about 3.5 m2 in a 1200 m2 building is a small
# perturbation, and it was measured before and after rather than assumed.
STATIC_TABLES = [t for _, _, t in COLLECTION_TABLES] + [DELIVERY_TABLE[1]]
