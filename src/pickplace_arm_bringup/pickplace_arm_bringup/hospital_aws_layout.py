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

# --- pickup: bench in the south wing, shelf facing west -----------------------
#
# lab_bench's shelf protrudes in -x, so yaw 0 presents it to a robot arriving
# from the west; the robot stands 1.53 m out at (-8.53, -26.50).
#   stand clearance 1.20 m, and connected - see below for why that word matters
#
# A BENCH IS TESTED ON A MAP THAT ALREADY CONTAINS THAT BENCH. This one used to
# stand at (8.50, -23.00), chosen against every measure in this file - wall
# clearance, prop clearance, room to reverse - and every one of them passed.
# It was still wrong, because all of them were computed on a map generated
# BEFORE the bench existed. Adding the bench to the map sealed the mouth of the
# bay it stands in: the robot's standing pose ended up in its own connected
# component, 29 against the rest of the building's 4.
#
# Nav2 drove in anyway - the local costmap only needs the inscribed radius to
# squeeze through - and then the global planner, which respects the full
# footprint, could not find a way back out. Every goal after the pick aborted,
# with the robot sitting level and undamaged in 0.75 m of clear space.
#
# So the check is: stamp the candidate bench into the map, THEN confirm the
# standing pose still shares a component with both the fleet ring and the
# delivery point. Three of the five candidates passed that; two did not.
BENCH_COLLECT = (-7.00, -26.50, 0.0)
# The rack sits 0.53 m out along the shelf, the offset hospital_lab uses.
RACK = (-7.53, -26.50)

# --- delivery: bench in the lobby, shelf facing south ------------------------
# The one pose that passed the back-off test unchanged: 0.69 m behind it.
BENCH_DELIVER = (-4.00, 9.00, 1.5707963)
DOCK = (-4.00, 8.50)
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
