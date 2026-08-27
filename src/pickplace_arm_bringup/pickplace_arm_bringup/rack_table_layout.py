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
# Distance from the table's origin to the face the robot approaches, MEASURED
# off aws_CoffeeTable_01_collision.DAE rather than taken as TABLE_SHORT/2. The
# collision mesh is a solid block y -0.330..+0.339 at every height from the
# floor to the top -- it is not a top on legs, and it is not symmetric. 0.330 is
# the number that matters: it is what the base has to stand clear of, and it is
# what the front camera sees when it measures the table directly (see
# _table_face_ahead in mission_delivery.py).
TABLE_NEAR_FACE = 0.330
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
# DELIVERY RACKS GO ON THE TABLE'S CENTRE LINE, NOT WHERE COLLECTION RACKS
# STAND. RACK_LOCAL_Y puts a rack 0.15 m in from the near edge, which is right
# for a collection table -- the rack is spawned there and has to be both seen
# and reached. Placing is a different problem, and reusing that offset failed
# it in two separate ways at once.
#
# ONE: 0.15 m of edge is less margin than the localisation has error. Where the
# arm puts the rack is computed from the robot's own pose, so it inherits
# AMCL's residual, and 0.26 m of that was measured at this very table. Run with
# the slots 0.15 m from the edge, ALL THREE racks were reported "delivered" and
# all three were found on the floor at the table's north edge:
#
#     rack_red   (-3.438, 10.340, z=0.063)     table top is z=0.324
#     rack_green (-3.018, 10.499, z=0.063)     slot targets were y=10.184
#     rack_blue  (-3.825, 10.592, z=-0.015)
#
# On the centre line the margin is the table's own half-depth, 0.334 m, in both
# directions -- more than twice the error that pushed them off.
#
# TWO: the outer slots were never actually in reach. They sit 0.30 m to either
# side, so from a 0.85 m standoff the arm had to make 0.850 m while its usable
# reach at that lateral offset and this height is 0.808 m. Every outer
# placement therefore depended on _close_in_on driving the last few centimetres
# -- a recovery, running as the normal path, on the two slots out of three that
# are hardest to hit.
#
# 0.75 m fixes both: 0.750 m of reach needed against 0.808 m available, and the
# robot still stands 0.416 m from the table edge -- 0.166 m of bumper clearance,
# and comfortably outside the 0.25 m inscribed zone that would make the standoff
# unplannable.
# THREE: THE STANDOFF WAS A POSE THE ROBOT CANNOT PHYSICALLY OCCUPY, and that
# is what was really wrong here. Everything above reasons about reach; nothing
# checked whether the base fits.
#
# rack_table's collision geometry is NOT a table with legs. Sliced every 2 cm
# from the floor to its top it is a SOLID BLOCK, x -0.657..+0.670,
# y -0.330..+0.339, at every height. So the robot cannot put any part of itself
# inside local y > -0.330, and a Husky's front bumper is 0.4937 m ahead of
# base_link. The nearest the base can stand is therefore
#
#     0.330 + 0.4937 = 0.8237 m from the table centre
#
# and the old numbers put it at 0.750 -- i.e. commanded 74 mm INSIDE the table.
# Nav2 was being asked to drive into a wall. It never arrived; it jammed
# wherever the contact stopped it, which is why the delivery approach needed
# relocalisations and retries, and why the reach cap then silently clamped the
# placement short and left the rack near the front edge.
#
# THE NUMBERS BELOW ARE SOLVED, NOT NUDGED. Writing R for the robot's local y,
# S for the slot's, D for the standoff and c for the clearance to the table:
#
#     R = S - D                      the robot sits D behind the slot
#     R + 0.4937 <= -0.330 - c       the bumper must clear the block
#     D <= reach(py)                 the arm must make it
#
# so the deepest reachable slot is S = D - 0.8237 - c, maximised by taking D as
# large as the arm allows. With ARM_REACH_FRACTION at 0.95 the arm reaches
# 0.835 m at the outer slots' 0.12 m lateral offset, so D = 0.81 leaves 25 mm of
# reach margin and c = 0.05 leaves 50 mm of bumper clearance:
#
#     slots      local y -0.065      robot centre local y -0.875
#     bumper     local y -0.381      table's face -0.330   -> 49 mm clear
#     rack face  local y -0.145      -> 185 mm of table in front of the rack
#
# THE RACK CANNOT GO DEEPER THAN THIS, and it is worth being explicit about why
# rather than leaving it to be re-litigated. Once the base stands clear of the
# block at 0.8237 m, reaching the table's CENTRE LINE needs 0.824 m of arm at
# the middle slot and 0.884 m at the outer ones, against an FR3 whose measured
# TCP limit is 0.855 m and which returned NO_IK_SOLUTION at 0.849. The far half
# of this table is not reachable by this robot at all. What was fixed is the
# variance, not the depth: the rack now lands at a repeatable 185 mm from the
# edge instead of wherever the reach clamp happened to stop it.
DELIVERY_SLOT_LOCAL_Y = -0.065
DELIVERY_NAV_STANDOFF = 0.81
DELIVERY_STANDOFF_LOCAL_Y = DELIVERY_SLOT_LOCAL_Y - DELIVERY_NAV_STANDOFF


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
# ONE horizontal slice at whatever height the LIDAR sits (0.4466 m when this
# search was run, 0.3143 m now), and one slice cannot describe a building full
# of things of different heights -- these tables are 0.32 m tall and were
# entirely underneath the old one. The first two placements this search produced
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
# COLLECT_0 MOVED WEST 0.40 m, FROM x=-8.50, AND IT IS THE SAME MISTAKE
# COLLECT_2 ALREADY TAUGHT THIS FILE.
#
# The lesson two blocks up says a robot needs 1.196 m to turn and that a gap the
# table narrowed to 1.25 m wedged it. Measured against the world's collision
# meshes, unioned over every height a robot occupies (0.05 to 1.15 m), collect_0
# stood in a 5.75 m bay with its clearances split:
#
#     west of the table   3.24 m
#     east of the table   1.19 m      <- against 1.196 m needed to turn
#
# i.e. the east side was 6 mm UNDER the turning requirement while the west side
# had three metres going spare, and the red robot kept hitting the table. That
# is not a tuning problem, it is the collect_2 failure a second time: a route
# that "exists" on a map eroded by the robot radius is not one Nav2 can drive
# with a real 0.99 x 0.67 m rectangle and a pose estimate with its own error.
#
# x=-8.90 rebalances the bay rather than just nudging the number:
#
#     west 2.84 m   east 1.59 m   standoff clearance 1.15 m
#     back-out strip 0.71 m       table footprint 0.38 m
#
# so the failing dimension goes from 0.00 m of margin to 0.39 m, and nothing
# else drops near its own limit (a robot needs 0.598 m of clearance to occupy a
# spot at all). Further west is worse, not better: the standoff and the back-out
# strip both shrink as the table approaches the room boundary west of it, and
# past x=-9.15 the back-out strip falls under 0.598 m, so the robot could reach
# the table and not get out again.
#
# THE STANDOFF MOVES WITH THE TABLE, which is why this fixes the approach and
# not the final few centimetres: everything in this module is derived from the
# table pose, so the robot still stops NAV_STANDOFF short of the same rack.
COLLECTION_TABLES = [
    ('collect_0', 'red',   (-8.90,  -5.00, math.pi)),      # west wing, north end
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
# 0.45, UP FROM 0.26, AND THE REASON IT COULD NOT BE THIS WIDE BEFORE IS THAT
# EVERY ROBOT USED TO PARK IN THE SAME PLACE.
#
# Every previous pass at this number asked "how far sideways can the arm reach
# from the ONE standoff in the middle of the table", and the answer bounded the
# spacing: at the placing depth of 0.81 m the arm covers +/-0.307 m of lateral
# offset, so slots further apart than that were simply unreachable and 0.26 was
# what fitted with a little margin. That left 0.084 m of nominal air between two
# 0.16 m racks -- and measured, with everything else working, red and green came
# to rest 0.065 m apart. Two racks that close touch as soon as anything is
# slightly off, and they did.
#
# THE SINGLE STANDOFF WAS THE CONSTRAINT, NOT THE ARM. A robot placing in the
# outer slot had to reach the full 0.26 m sideways PLUS whatever its own lateral
# error was -- up to 0.52 m, well outside the envelope -- while a robot parked
# in front of its OWN slot only has to reach its residual error. So the robot
# now drives to a per-slot standoff (see delivery_slots) and the lateral budget
# is spent on error instead of on geometry, which is what lets the row open up.
#
# 0.38, PULLED BACK IN FROM 0.45, AND THE NUMBER THAT DECIDES IT IS THE EDGE
# MARGIN RATHER THAN THE GAP.
#
# The binding limit is the table: the top is 1.327 m long, so with a rack
# 0.16 m wide the outer slot's far edge sits at SPACING + 0.089 and whatever is
# left over is all that stands between a placement error and a rack hanging off
# the end. At 0.45 that leftover was 0.124 m, and a measured outer placement
# came in 0.212 m along the row from its slot -- the rack stayed on the table
# only because half of it was still over the top, balanced on the edge.
#
#     spacing   outer rack edge   table left over   air between racks
#       0.45         0.539             0.124              0.274
#       0.38         0.469             0.194              0.204
#
# 0.38 buys 0.070 m of edge margin for 0.070 m of gap, and the gap is the thing
# there is plenty of: 0.204 m of air against the 0.084 m the row started with,
# i.e. still two and a half times the original margin, while the edge margin
# now covers the largest error ever recorded here with 0.019 m to spare once
# the lateral correction is only trusted where it is well conditioned (see
# NEAR_EDGE_MAX_BEARING in mission_delivery).
#
# THIS IS THE "BRING THE OUTER RACKS IN A LITTLE" CHANGE. Both outer slots move
# 0.07 m toward the middle; the middle slot does not move.
DELIVERY_SLOT_SPACING = 0.38
# Left, middle, right as the robot facing the table sees them: local +x is the
# robot's right (base_link y = -(local x - robot's local x), see the frame
# convention at the top of this file). So slot 0 is the LEFT-hand slot, slot 2
# the right-hand one, and the manager hands them out in that order.
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

    THE PER-SLOT ROBOT POSE IS A DRIVING INSTRUCTION AGAIN, and the objection
    that made it geometry-only has expired. It used to read: a robot sent to
    one of these standoffs arrives 0.34 to 0.47 m from the pose it was given,
    which is LARGER THAN THE SLOT SPACING, so driving to a slot's standoff
    cannot address that slot -- the robot may end up squarely in front of its
    neighbour. Everything in that is still true about the DRIVE. It stopped
    mattering when the placement stopped being computed from the drive.

    Which slot is which is now anchored to the TABLE, not to the robot: the
    lidar fits the table's front face, the camera measures its depth, and the
    slot is built in that face's own frame (see _slot_on_face in
    mission_delivery.py). A robot that arrives half a metre off still knows
    exactly where all three slots are -- it just has to reach a little further
    sideways.

    So the standoff no longer decides WHICH slot; it decides HOW FAR the arm has
    to reach. Parking once in the middle spent the whole lateral budget on the
    slot offset and left none for error, which capped the row at 0.26 m and put
    two racks 0.065 m apart. Parking in front of the slot spends it all on
    error: the nominal lateral offset is zero, and the arm's +/-0.307 m at the
    placing depth is then pure margin against a residual measured at 0.09 to
    0.26 m.

    Clearance at the three standoffs, measured against maps/aws_hospital.pgm
    with the delivery table's own footprint excluded: 3.08, 2.66 and 2.25 m.
    """
    _, table = DELIVERY_TABLE
    out = []
    for i, lx in enumerate(DELIVERY_SLOT_LOCAL_X):
        slot = _to_world(table, lx, DELIVERY_SLOT_LOCAL_Y)
        stand = _to_world(table, lx, DELIVERY_STANDOFF_LOCAL_Y)
        out.append((i, slot, stand + (_robot_yaw(table),)))
    return out


def delivery_standoff(slot_index=None):
    """The pose a robot drives to in order to use the delivery table.

    With a slot index, the standoff directly in front of THAT slot -- which is
    what a robot about to place should ask for, so the arm's lateral reach is
    spent on its own error rather than on the width of the row. See
    delivery_slots() for why that changed.

    With no index, the standoff centred on the table: the neutral pose for
    anything that has to be at the table without placing in a particular slot.
    """
    _, table = DELIVERY_TABLE
    lx = 0.0 if slot_index is None else DELIVERY_SLOT_LOCAL_X[slot_index]
    return _to_world(table, lx, DELIVERY_STANDOFF_LOCAL_Y) + (_robot_yaw(table),)


def delivery_table_pose():
    return DELIVERY_TABLE[1]


# --- where robots WAIT for the delivery table ---------------------------------
#
# A RESERVED BEARING ON A CIRCLE, ONE PER ROBOT. Each robot drives toward the
# delivery table and stops at ITS OWN point on the DELIVERY_HOLD_RADIUS circle,
# facing the table.
#
# 3.0 m is chosen against the geometry it has to clear: the delivery standoff
# sits 0.75 m from the table centre and the placing robot needs room to turn
# there, so holding at 3.0 m leaves 2.25 m between a waiting robot and the one
# working -- comfortably more than the 1.20 m two Husky circumscribed radii
# need, with margin for the localisation error that put a robot 1.25 m from
# where it believed it was.
DELIVERY_HOLD_RADIUS = 3.00

# THE BEARING IS RESERVED, NOT DERIVED FROM THE APPROACH, AND THAT IS THE WHOLE
# POINT OF THIS BLOCK.
#
# It used to be derived: the robot stopped wherever the line from its own
# position to the table crossed the circle. The argument for it was that "the
# three collection tables are in different parts of the building, so the three
# approach bearings are naturally spread around the circle". THAT IS FALSE FOR
# THIS LAYOUT, and it is false in the one way that matters. All three
# collection tables are SOUTH of the lobby, so all three robots arrive from the
# south, and two of them arrive from almost the same bearing:
#
#     r1 / red    from collect_0   bearing -111.8 deg   hold (-4.613, 7.214)
#     r3 / blue   from collect_2   bearing  -98.5 deg   hold (-3.946, 7.033)
#     r2 / green  from collect_1   bearing  -68.8 deg   hold (-2.417, 7.202)
#
# red and blue are 13.3 degrees apart on a 3 m circle, i.e. 0.691 m between the
# two poses -- against the 1.196 m that two Husky circumscribed radii need. The
# fleet was therefore SENT TO TWO OVERLAPPING POSES: not a near miss the local
# costmap could resolve, but two goals whose chassis interpenetrate, both held
# until the manager's rendezvous released them. The two robots ground into each
# other for the whole wait. (green was safe by luck, at 2.20 and 1.54 m.)
#
# Deriving the bearing cannot be repaired by widening the circle either: two
# robots arriving 13 degrees apart need R >= 6.0 m to be 1.6 m apart, which is
# through the lobby's west wall, and nothing bounds the angle anyway -- two
# identical bearings stay identical at any radius. The angle has to be reserved.
#
# CHOSEN BY MEASUREMENT AGAINST maps/aws_hospital.pgm, the same way the fleet
# formation and the collection tables were. Every bearing on the circle was
# scored for clearance to the nearest occupied cell, and triples were searched
# for the one maximising the WORST clearance subject to every pair being at
# least 1.60 m apart, keeping the arc order the same as the natural approach
# order so no robot's run in crosses another's:
#
#     index 0   r1 / red     -116 deg   (-4.815, 7.304)   1.83 m clear
#     index 2   r3 / blue     -85 deg   (-3.239, 7.011)   1.80 m clear
#     index 1   r2 / green    -43 deg   (-1.306, 7.954)   1.81 m clear
#
#     red <-> blue 1.60 m    blue <-> green 2.15 m    red <-> green 3.57 m
#
# i.e. 0.40 m of daylight past the 1.196 m requirement at the tightest pair, and
# 1.20 m past the 0.598 m circumscribed radius at the worst pose. The west side
# of the circle is a wall (clearance falls under 0.6 m past -135 deg) and is not
# used; every bearing here is on the south side the robots arrive from, so
# nobody crosses the front of the table to reach its hold pose. Each is 2.9 to
# 3.8 m from the delivery standoff, so a waiting robot is never in the way of
# the working one.
#
# INDEXED THE SAME WAY ARM_ROBOTS IS, which is what makes the reservation
# race-free: assignment is static, so two robots can never choose the same spot.
DELIVERY_HOLD_BEARINGS = (
    math.radians(-116.0),      # index 0 -- r1, arriving from collect_0
    math.radians(-43.0),       # index 1 -- r2, arriving from collect_1
    math.radians(-85.0),       # index 2 -- r3, arriving from collect_2
)


def delivery_hold_pose(index):
    """Where robot `index` waits for its turn at the delivery table.

    Returns (x, y, yaw) on the DELIVERY_HOLD_RADIUS circle at that robot's
    reserved bearing, facing the table -- so it is already pointing the right
    way when its turn comes. See DELIVERY_HOLD_BEARINGS for why the bearing is
    reserved per robot rather than taken from the approach direction.
    """
    tx, ty, _ = DELIVERY_TABLE[1]
    a = DELIVERY_HOLD_BEARINGS[index % len(DELIVERY_HOLD_BEARINGS)]
    ux, uy = math.cos(a), math.sin(a)
    return (tx + ux * DELIVERY_HOLD_RADIUS,
            ty + uy * DELIVERY_HOLD_RADIUS,
            math.atan2(-uy, -ux))


# --- what belongs in the map --------------------------------------------------
#
# ALL FOUR TABLES, and this is a genuine trade-off rather than an obvious call.
#
# THE ARGUMENT AGAINST HAS SINCE EXPIRED: a table top at 0.3238 m was BELOW the
# LIDAR's old 0.4466 m scan plane, so the sensor could never return a point on
# one, and stamping them put occupied cells in the map that no measurement would
# ever support. Hung under the top plate the scanner cuts these tables at
# 0.3143 m, 9.5 mm under their tops, so those cells are now measured like any
# other. It was worth stamping even when it was not, for the reason below.
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
