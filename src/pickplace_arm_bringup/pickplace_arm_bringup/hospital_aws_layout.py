"""Where the sample run's props stand in aws_hospital.sdf.

ONE SOURCE OF TRUTH, IMPORTED BY THREE THINGS: the mission that drives to these
places, the launch that spawns them, and the map generator that has to put them
in the map. They fell out of sync exactly once and it cost a debugging session -
the launch spawned a bench the map knew nothing about, Nav2's global planner
routed the robot straight through it, and the robot ended up wedged against a
bench that from the planner's point of view was thin air.

DELIBERATELY FREE OF ROS IMPORTS. The map generator runs as a plain script and
must not drag in pymoveit2 and rclpy just to learn where a bench is.

Poses are (x, y, yaw) in the world frame, which is also the map frame here.
"""

# EVERY POSE BELOW IS JUDGED ON THE SPACE THE ROBOT REVERSES INTO, not just on
# the space it stands in. That distinction cost a whole debugging session.
#
# The first collect bench scored well on the obvious measures - 1.15 m of
# clearance at the standing pose, connected to the rest of the building - and
# was still wrong, because it sat inside a curtained bed bay. The robot drove
# there, picked the rack up, and then reversed into the curtain: it ended up
# 0.8 m away at a spot with 0.45 m of clearance, which is less than the Husky's
# own 0.598 m circumscribed radius. That cell is not navigable, so Nav2 had no
# valid START pose, and planner_server aborted every goal without logging why.
# The robot sat there at an 11 degree tilt for the rest of the run.
#
# So each standing pose is checked over the 2.0 x 1.4 m strip it backs out
# through, and the figures below are the worst clearance anywhere in that strip.

# --- pickup: bench placed by hand in the south wing, shelf facing east --------
#
# THIS POSE WAS CHOSEN IN THE GUI, NOT BY SEARCH. The user dragged the bench to
# exactly here and asked for it to stay, so these numbers are read back off the
# running world (/world/aws_hospital/pose/info) rather than computed. Do not
# "improve" them - a clearance search will happily propose somewhere else, and
# somewhere else is not the room that was asked for.
#
# THE ROOM MATTERS, NOT JUST THE GEOMETRY. The bench used to sit at
# (-7.00, -26.50), which is clear, reachable and completely wrong: that room is
# the staff rest room. Blood samples are not collected there. The building has
# meaning that no clearance metric can see, which is exactly why the placement
# ended up being made by hand.
#
# lab_bench's shelf protrudes in -x, so yaw pi turns it to face +x: the shelf,
# the rack on it and the robot's standing pose are all east of the bench origin.
BENCH_COLLECT = (-3.015, -27.172, 3.1415927)
# The rack sits 0.53 m out along the shelf, the offset hospital_lab uses, which
# with yaw pi is +x in the world. It stands ON the shelf at z 0.30 - spawned at
# the wrong place once, it simply lay on the floor beside the bench and the
# grasp height was 0.30 m too high.
RACK = (-2.485, -27.172)

# --- delivery: bench at the information desk, shelf facing north --------------
#
# ALSO PLACED BY HAND, then squared up. The bench was dropped in at
# (-0.098, 0.461) with 6.8 deg of pitch, 2.1 deg of roll, 0.274 m of float and a
# yaw of -93.3 - a GUI drag, not a considered pose. Tidying it is arithmetic
# against two measured edges, not taste:
#
#   the information desk's north face runs dead straight along y = 0.000,
#   sampled every 0.5 m from x -2.0 to +2.0 and identical at every one.
#
#   lab_bench at yaw -pi/2 maps its local +x (the worktop's back edge, +0.38)
#   to world -y, so the bench's back sits 0.38 m south of its origin.
#
# y 0.50 therefore leaves a clean 0.12 m gap to the counter, x 0.00 centres the
# 2.06 m carcass on the desk, and the shelf - local x -0.65..-0.35 - comes out
# at world y 0.85..1.15, facing north into the open lobby where the robot has
# room to approach and turn.
BENCH_DELIVER = (0.00, 0.50, -1.5707963)
# The dock stands 0.50 m out along the shelf, which is +y at this yaw. That puts
# it at the exact centre of the shelf slab (0.85..1.15) rather than perched on
# an edge, and PROP_SPAWN_Z lands it on the 0.30 m shelf top.
DOCK = (0.00, 1.00)
DOCK_TOP = 0.38

# --- the handover -------------------------------------------------------------
#
# Two robots nose to nose 1.55 m apart, inside the 1.24 .. 1.87 m window where
# both arms reach the point between them: the FR3 reaches 0.935 m ahead of its
# own base_link, and two chassis closer than 1.24 m touch.
#
# The previous pair, at (8.75, -6.75), passed on standing clearance and failed
# this rule badly - 0.45 m behind the carrier and 0.05 m behind the receiver.
# The handover would have deadlocked both robots the same way the pick did.
# Here both can reverse out through 0.80 m, and the midpoint still splits the
# route almost evenly: 18.5 m for the carrier, 18.1 m for the receiver.
HANDOVER_CARRIER = (8.00, -5.28, 1.5707963)     # faces north, at the receiver
HANDOVER_RECEIVER = (8.00, -3.73, -1.5707963)   # faces south, at the carrier

# WHERE THE RACK CHANGES HANDS: THE FLOOR BETWEEN THEM, NOT JAW TO JAW.
#
# The reach window above is what makes this point usable, and it is exactly the
# midpoint - 0.775 m ahead of each robot's base_link, well inside both arms'
# envelope (0.727 m from the arm base, against the FR3's 0.855 m reach) and
# inside MAX_REACH_X = 0.85 for both.
#
# THE TRANSFER IS GROUND-TO-GROUND BECAUSE BOTH GRIPPERS POINT STRAIGHT DOWN.
# Every arm motion in this package holds zdown from pick through carry through
# place - see the note on fixed wrist orientation in mission_2.py - so two
# grippers meeting on the same 0.06 m grip block would have to occupy the same
# space from the same direction. Passing it jaw to jaw needs at least one robot
# to grasp from the side, which is a new arm envelope, a new IK branch and a new
# collision problem between two arms that cannot see each other.
#
# Setting it down costs one extra lower-and-lift and buys the whole handover out
# of code paths that are already flown on every run: the carrier's set-down is
# place_on_column's lower/detach/release/retreat with the target height set to
# the floor, and the receiver's pick is the ordinary claw pick with grasp_z =
# PAYLOAD_FLOOR_Z. Neither robot learns anything new.
#
# It also removes the tightest timing constraint in the alternative. Held jaw to
# jaw, the rack is only ever supported by one weld at a time and the window
# between the receiver welding and the carrier releasing is the whole safety
# margin. On the floor there is no window: the rack stands on its own tray, and
# the two robots can take as long as they like.
HANDOVER_POINT = ((HANDOVER_CARRIER[0] + HANDOVER_RECEIVER[0]) / 2.0,
                  (HANDOVER_CARRIER[1] + HANDOVER_RECEIVER[1]) / 2.0)

# --- getting there: the carrier's route needs one waypoint --------------------
#
# THE MEET POSES ARE FINE. THE ROUTE TO THEM IS NOT, AND ONLY THE RELAY DRIVES
# IT. The single-robot run goes bench -> dock and never comes this way, so this
# corridor was never driven until stage 5 existed.
#
# WHAT HAPPENS WITHOUT THIS WAYPOINT, twice out of two runs. Leaving the collect
# bench for HANDOVER_CARRIER, the carrier tracks east along y ~ -27.3, sails
# past the northward turn, and drives into a dead-end pocket east of it. Ground
# truth at the stop, from `gz model -p`: (9.025, -26.568) on the first run and
# (8.906, -26.280) on the second - three identical samples in a row, i.e. not
# moving. Measured against this package's own map, that pocket profiles as
#
#   x  8.5   clearance 0.85 m
#   x  9.0   clearance 0.35 m     <- where it stops
#   x  9.5+  clearance 0.00 m     <- wall
#
# and 0.35 m is below the Husky's 0.598 m circumscribed radius, so the cell is
# not navigable. From there Nav2 has no valid start pose and every recovery
# refuses to run - "Running backup -> Collision Ahead - Exiting DriveOnHeading
# -> backup failed" - which is the same signature this package already
# documents for the curtained bed bay. The wheels then spin against the wall,
# odometry accumulates travel the robot never made, and AMCL follows it: 12.2 m
# of divergence (truth 9.025,-26.568 against amcl 6.135,-14.725) while the robot
# stood perfectly still. r2, driving normally at the same moment, was 0.20 m out.
#
# WHY THE PLANNER GOES THERE AT ALL. NavFn only refuses LETHAL cells, and
# nav2_params.yaml sets inflation_radius to 0.36 - just 0.015 m above the 0.345 m
# inscribed radius Nav2 computes for this footprint. That is deliberate and
# documented there (0.45 stopped NavFn extracting paths through the 1.5 m
# doorways), but its cost is that a gap the robot cannot fit through never
# becomes lethal, so nothing stops a path being drawn into it.
#
# AND THE REAL OBSTACLE IS A DOORWAY, WHICH IS WHY THERE ARE TWO WAYPOINTS AND
# WHY BOTH SIT ON THE SAME LINE.
#
# The collect bench stands in an east-west bay closed off to the north by the
# wall along y = -26.0. There is exactly one way out of that bay toward the
# lobby: a gap in that wall, measured off this package's own map as running
#
#   x 4.70 .. 6.15        i.e. 1.45 m of opening
#
# which sounds ample and is not, because clearance is measured to the NEAREST
# wall in any direction and the gap has jambs on both sides:
#
#   x 4.80   clearance 0.15 m        x 5.60   clearance 0.55 m
#   x 5.00   clearance 0.35 m        x 5.80   clearance 0.35 m
#   x 5.20   clearance 0.50 m        x 6.00   clearance 0.15 m
#   x 5.45   clearance 0.75 m   <- the widest line
#
# The Husky's circumscribed radius is 0.598 m, so the band where it fits at ANY
# yaw is only x 5.30 .. 5.55 - a quarter of a metre inside a 1.45 m doorway.
# Aim anywhere else and the robot has to be near-perfectly square to the gap.
#
# WHAT THAT LOOKS LIKE FROM OUTSIDE is a robot that cannot get out of the room.
# Observed directly: it ran east to x = 4.85, backed off west to x = 1.66, came
# forward again, over and over, along y ~ -27.4 - up to the doorway and away
# from it, never through. Nav2's global planner is happy to aim into the gap
# because inflation_radius is 0.36 (see nav2_params.yaml) and everything from
# x 5.06 to 5.79 is therefore un-inflated and looks free; the local controller,
# which checks the true 0.99 x 0.67 m footprint, then refuses at the jamb.
#
# THE FIRST WAYPOINT IS AN ALIGNMENT, NOT A DESTINATION. Sending one goal north
# of the door still lets the robot arrive at the gap crabbed, because nothing
# constrained where it entered from. Two goals on the SAME x line - one back in
# the bay, one out the far side - make the traverse a straight run up x = 5.45,
# and that line clears 0.75 m at its worst point anywhere between y -28.0 and
# -24.0, i.e. above the circumscribed radius for the whole passage.
#
# 5.45 is the measured argmax of clearance across the opening, not the midpoint
# of the jambs (5.42) and not a number read off a route printout. An earlier
# version of this used 5.20, taken from an A* route sampled every 2 m, and 5.20
# is 0.10 m below the circumscribed radius - inside the doorway but not inside
# the part of it the robot fits through.
#
# Sent as ordinary goals before the meet pose rather than through
# NavigateThroughPoses, so each inherits navigate_to's retries and costmap
# clearing.
_DOOR_X = 5.45
HANDOVER_VIA = [
    (_DOOR_X, -27.25, 1.5707963),   # in the bay, squared up on the doorway
    (_DOOR_X, -24.60, 1.5707963),   # through it and clear, still facing north
]

# --- what belongs in the map --------------------------------------------------
#
# The benches and the dock never move, so the global planner must know about
# them or it will plan through them. The RACK is deliberately absent: the robot
# picks it up and carries it away, and a map obstacle that is no longer there is
# worse than one that was never drawn.
STATIC_PROPS = [
    ('lab_bench', BENCH_COLLECT),
    ('lab_bench', BENCH_DELIVER),
    ('delivery_dock', (DOCK[0], DOCK[1], 0.0)),
]
# Spawn z is the model origin. lab_bench sits on the floor; the dock's origin is
# on its own underside, so it stands at shelf height.
PROP_SPAWN_Z = {'lab_bench': 0.0, 'delivery_dock': 0.30, 'sample_rack': 0.30}


# --- the fleet ----------------------------------------------------------------
#
# Here rather than in the launch file because the map generator needs it too and
# must not import launch machinery. (namespace, x, y, yaw); the ring is fixed -
# see the launch for how it was measured and why it must not move.
ROBOTS = [
    ('r1', -2.25, 11.50, 0.0),
    ('r2', 2.25, 11.50, 3.14159265),
    ('r3', 0.0, 13.75, -1.5707963),
    ('r4', 0.0, 9.25, 1.5707963),
]
# Which of them navigate, and so which of them MOVE. Everything else in the ring
# is furniture as far as a planner is concerned.
NAV_ROBOTS = ('r1', 'r2')
ARM_ROBOTS = ('r1', 'r2')

# WHO RUNS WHICH LEG OF THE RELAY. Both must be in ARM_ROBOTS and NAV_ROBOTS:
# the carrier picks and drives, the receiver drives and places. Named here, in
# the ROS-free layout module, because the relay launch file needs them and must
# not import the mission stack (pymoveit2, rclpy and a MoveIt config) just to
# learn two namespaces.
#
# r1 carries because it is the robot the single-robot run already uses, so the
# collect leg is the leg that has been flown; r2 receives. The pair is otherwise
# symmetric - they are identical robots facing each other across the ring.
RELAY_CARRIER_NS = 'r1'
RELAY_RECEIVER_NS = 'r2'

# A200 chassis footprint, base_link at the chassis origin: bumper at +0.494,
# tail at -0.496, half-width 0.335. The same rectangle nav2_params.yaml gives
# the costmaps.
ROBOT_FOOTPRINT = (0.494, -0.496, 0.335)


def parked_robots():
    """The fleet members that never move, as (x, y, yaw).

    THE PARKED ROBOTS BELONG IN THE MAP. They are 0.99 x 0.67 m of solid
    chassis standing in fixed positions, and r4 sits directly in the ring's
    southern exit. Left out of the map, the global planner routes straight
    through it, exactly as it did through the nurses' station and the delivery
    bench - and the mission robot drove out of the ring, stopped 1.03 m from r4,
    and never moved again through three runs.

    r1 and r2 are excluded because they drive: a map obstacle where a robot used
    to be is worse than no obstacle at all.
    """
    return [(x, y, yaw) for ns, x, y, yaw in ROBOTS if ns not in NAV_ROBOTS]
