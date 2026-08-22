"""Where the fleet stands in aws_hospital.sdf.

ONE SOURCE OF TRUTH, IMPORTED BY EVERYTHING THAT NEEDS TO KNOW WHERE A ROBOT
IS: the launch that spawns them, the AMCL seed for each, and later the task
manager that dispatches them. Duplicating a pose here into a second file is the
single easiest way to lose an afternoon in this project, and it has been lost
that way before -- a prop spawned where the map knew nothing about it, and the
global planner routed straight through it.

DELIBERATELY FREE OF ROS IMPORTS, so an offline clearance check or the map
generator can read it without dragging in rclpy and pymoveit2.

Poses are (x, y, yaw) in the world frame, which is also the map frame here:
maps/aws_hospital.yaml is generated from the world's own geometry by
aws_hospital_map.py, so map == world exactly.
"""
import math

# --- the reception desk -------------------------------------------------------
#
# MEASURED FROM THE COLLISION MESH, NOT THE MODEL ORIGIN, and the gap between
# those two is the whole reason this block exists.
#
# aws_robomaker_hospital_nursesstation_01 is included at pose (0, 1.5). Its
# collision mesh occupies
#
#     x -3.50 .. +3.50      y +2.80 .. +6.30      z -0.02 .. +1.14
#
# so the desk's origin sits 1.3 m SOUTH of the nearest geometry and 4.8 m south
# of the far edge. A clearance check written against the origin therefore reads
# "5 m clear" at a point that is inside the desk. That is not hypothetical: an
# earlier fleet ring in this project put a robot at (0, 5.00) on exactly that
# reasoning and parked it behind the reception counter.
#
# Two members of staff stand behind the counter at (-1.05, 4.49) and
# (1.07, 4.51), which is what confirms the counter's public side is the NORTH
# one, facing the lobby.
DESK_BOUNDS = (-3.50, 3.50, 2.80, 6.30)     # xmin, xmax, ymin, ymax
DESK_FACE_Y = 6.30                          # the front (lobby-facing) edge
DESK_FACE = (0.0, DESK_FACE_Y)              # centre of that edge

# --- the formation ------------------------------------------------------------
#
# An equilateral triangle in the lobby, in front of the desk. Centre, radius and
# rotation are all that define it; the three poses fall out below, so changing
# the formation is changing these three numbers.
#
# CHOSEN BY SEARCH, THEN RE-MEASURED IN 3D. Every centre on a 0.25 m grid, four
# circumradii and twelve rotations were scored against maps/aws_hospital.pgm,
# keeping only formations where all three vertices stand on free floor with at
# least 1.20 m of clearance -- twice the Husky's 0.598 m circumscribed radius.
# Maximising clearance alone is the wrong objective and was tried first: it
# walks the triangle into the middle of the lobby, 3.65 m off the desk, which is
# not "in front of reception" in any useful sense. So the survivors were ranked
# by how CLOSE the formation stands to the desk face instead.
#
# The winner was then re-checked against the world's collision meshes sliced at
# 5 cm steps from 0.05 m to 1.15 m, because the map is a single horizontal slice
# at the LIDAR's 0.4466 m and cannot see a low table or the lip of a counter:
#
#     r1 (apex, north)   4.50 m clear
#     r2 (front left)    1.74 m clear
#     r3 (front right)   1.74 m clear
#
# i.e. 1.14 m of daylight past the circumscribed radius at the tightest pose,
# with the front rank standing 1.45 m off the counter.
#
# RADIUS 2.00 RATHER THAN 1.70. The two score within 0.02 m of each other on
# clearance, but 2.00 puts 3.46 m between neighbouring robots against 2.94 m --
# and three robots that will leave this formation at the same moment want the
# separation more than they want the extra 15 cm of counter standoff.
FORMATION_CENTRE = (0.0, 8.75)
FORMATION_RADIUS = 2.00
FORMATION_ROTATION = math.pi / 2.0      # apex points north, away from the desk


def _vertex(k):
    """Vertex k of the formation triangle, k in 0..2."""
    a = FORMATION_ROTATION + k * 2.0 * math.pi / 3.0
    return (FORMATION_CENTRE[0] + FORMATION_RADIUS * math.cos(a),
            FORMATION_CENTRE[1] + FORMATION_RADIUS * math.sin(a))


def _facing(p, target):
    """Yaw that points a robot standing at p at `target`."""
    return math.atan2(target[1] - p[1], target[0] - p[0])


# EVERY ROBOT LOOKS AT THE RECEPTION COUNTER, which is what makes this a fleet
# stationed at reception rather than three robots that happen to form a
# triangle. It is also one line to change if a later step wants them facing
# their first destination instead.
#
# The naming is positional and stable: r1 is the apex, r2 and r3 are the front
# rank left and right as seen from the desk.
ROBOTS = [
    ('r1',) + _vertex(0) + (_facing(_vertex(0), DESK_FACE),),
    ('r2',) + _vertex(1) + (_facing(_vertex(1), DESK_FACE),),
    ('r3',) + _vertex(2) + (_facing(_vertex(2), DESK_FACE),),
]

# Every robot in this fleet drives AND manipulates -- each one collects a rack
# and delivers it. That is a deliberate difference from the abandoned four-robot
# ring, where two robots were parked furniture because the machine could not run
# four navigation stacks and four move_groups at once. Three of each is what
# this plan needs; watch the real-time factor, because if it falls far enough
# the LIDAR starves and AMCL follows it down.
NAV_ROBOTS = tuple(ns for ns, *_ in ROBOTS)
ARM_ROBOTS = tuple(ns for ns, *_ in ROBOTS)

# base_link (the A200 chassis origin) sits 0.13228 m up so the wheels touch the
# floor; the small extra margin lets the robot settle onto the floor rather than
# spawning interpenetrating it.
SPAWN_Z = 0.14
WORLD_ENTITY = 'aws_hospital'

# A200 chassis footprint, base_link at the chassis origin: bumper at +0.494,
# tail at -0.496, half-width 0.335 -- the same rectangle nav2_params.yaml gives
# the costmaps. Circumscribed radius 0.598, inscribed 0.335.
ROBOT_FOOTPRINT = (0.494, -0.496, 0.335)
ROBOT_RADIUS = 0.598


def robot(ns):
    """The (x, y, yaw) of one robot by namespace."""
    for name, x, y, yaw in ROBOTS:
        if name == ns:
            return (x, y, yaw)
    raise KeyError(f'{ns} is not in the fleet: {[r[0] for r in ROBOTS]}')
