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
