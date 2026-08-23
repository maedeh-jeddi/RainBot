"""Generate one RViz config for a whole fleet, from the fleet's own robot list.

WHY THIS IS GENERATED AND NOT A COMMITTED .rviz FILE. config/mission.rviz and
config/navigation.rviz both describe ONE robot on unprefixed topics - /scan,
/plan, /odometry/filtered - and there is no way to point them at four robots at
once. Written out by hand for the fleet, the file would be about thousands of
lines of near-duplicate YAML with the namespace as the only difference, and the
next robot added to ROBOTS in hospital_fleet.launch.py would silently not
appear in it. Generating it from that same list keeps the promise the fleet
launch file makes: adding a robot is one line, everywhere.

The output is written to a temp file and handed to `rviz2 -d`, exactly as
ns_params.py does for the Nav2 and AMCL parameter files.

WHAT IS ON BY DEFAULT AND WHY. RViz starts while Gazebo is still streaming 175
models in, on a machine the fleet launch file already documents as running near
its limit (load 23.95 measured, sixteen cores). So the displays that cost real
CPU per frame are shipped DISABLED - the point clouds above all, at roughly
5 MB per message per camera - and the cheap ones that answer "is it working?"
are shipped enabled: the map, each robot's model, its LIDAR, its plan and its
camera image. Tick the rest on in the Displays panel when you actually want
them; nothing needs relaunching.
"""

import os
import tempfile

import yaml

# One colour per robot, used for its LIDAR, its odometry arrows and its
# footprint, so the whole fleet can be told apart at a glance in one view.
COLOURS = ['255; 60; 60', '60; 255; 60', '60; 160; 255', '255; 200; 0']

# QoS blocks. Sensor data comes off ros_gz_bridge and is read best-effort; the
# map and the costmaps are latched, and a Volatile subscriber on a
# Transient Local publisher gets NOTHING if it subscribes after the one message
# was published - which is always the case here, because RViz is up long before
# map_server is.
_SENSOR_QOS = {'Depth': 5, 'Durability Policy': 'Volatile',
               'History Policy': 'Keep Last',
               'Reliability Policy': 'Best Effort'}
_RELIABLE_QOS = {'Depth': 5, 'Durability Policy': 'Volatile',
                 'History Policy': 'Keep Last',
                 'Reliability Policy': 'Reliable'}
_LATCHED_QOS = {'Depth': 5, 'Durability Policy': 'Transient Local',
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Reliable'}


def _with_filter(qos):
    """Topic blocks on displays carry a Filter size; tool topics do not."""
    return dict(qos, **{'Filter size': 10})


def _grid():
    return {
        'Alpha': 0.5, 'Cell Size': 1, 'Class': 'rviz_default_plugins/Grid',
        'Color': '100; 100; 100', 'Enabled': True,
        'Line Style': {'Line Width': 0.03, 'Value': 'Lines'},
        'Name': 'Grid', 'Normal Cell Count': 0,
        'Offset': {'X': 0, 'Y': 0, 'Z': 0}, 'Plane': 'XY',
        'Plane Cell Count': 60, 'Reference Frame': '<Fixed Frame>',
        'Value': True,
    }


def _map(name, topic, scheme, alpha, enabled):
    return {
        'Alpha': alpha, 'Class': 'rviz_default_plugins/Map',
        'Color Scheme': scheme, 'Draw Behind': True, 'Enabled': enabled,
        'Name': name,
        'Topic': _with_filter(_LATCHED_QOS) | {'Value': topic},
        'Update Topic': _with_filter(_RELIABLE_QOS) | {'Value': topic + '_updates'},
        'Use Timestamp': False, 'Value': enabled,
    }


def _tf(enabled):
    # SHIPPED OFF. Four robots is four full TF trees - the FR3 alone is a dozen
    # links each - and drawing all of them at once buries the map under arrows.
    # Tick it on and use the Frames list to pick out the one tree you want.
    return {
        'Class': 'rviz_default_plugins/TF', 'Enabled': enabled,
        'Filter (blacklist)': '', 'Filter (whitelist)': '',
        'Frame Timeout': 15, 'Frames': {'All Enabled': True},
        'Marker Scale': 0.5, 'Name': 'TF', 'Show Arrows': True,
        'Show Axes': True, 'Show Names': False, 'Tree': {},
        'Update Interval': 0, 'Value': enabled,
    }


def _robot_model(ns):
    # TF Prefix is what makes a shared-URDF fleet work in one RViz. The
    # description on /<ns>/robot_description names its links `base_link`,
    # `fr3_link0` and so on with no prefix, exactly as move_group needs them
    # (see the MoveIt comment in hospital_fleet.launch.py), while the TF
    # tree the robot actually publishes is prefixed by robot_state_publisher's
    # frame_prefix. Without this, every robot's model is looked up in the
    # unprefixed tree, no transform is found, and all four render nothing at
    # all with "No transform from [base_link] to [map]".
    return {
        'Alpha': 1, 'Class': 'rviz_default_plugins/RobotModel',
        'Collision Enabled': False, 'Description File': '',
        'Description Source': 'Topic',
        'Description Topic': dict(_LATCHED_QOS, **{'Value': f'/{ns}/robot_description'}),
        'Enabled': True, 'Links': {'All Links Enabled': True,
                                   'Expand Joint Details': False,
                                   'Expand Link Details': False,
                                   'Expand Tree': False,
                                   'Link Tree Style': 'Links in Alphabetic Order'},
        'Mass Properties': {'Inertia': False, 'Mass': False},
        'Name': f'{ns} model', 'TF Prefix': ns, 'Update Interval': 0,
        'Value': True, 'Visual Enabled': True,
    }


def _laser(ns, colour):
    return {
        'Alpha': 1, 'Autocompute Intensity Bounds': True,
        'Autocompute Value Bounds': {'Max Value': 10, 'Min Value': -10,
                                     'Value': True},
        'Axis': 'Z', 'Channel Name': 'intensity',
        'Class': 'rviz_default_plugins/LaserScan', 'Color': colour,
        'Color Transformer': 'FlatColor', 'Decay Time': 0, 'Enabled': True,
        'Invert Rainbow': False, 'Max Color': '255; 255; 255',
        'Max Intensity': 4096, 'Min Color': '0; 0; 0', 'Min Intensity': 0,
        'Name': f'{ns} LIDAR', 'Position Transformer': 'XYZ',
        'Selectable': True, 'Size (Pixels)': 3, 'Size (m)': 0.04,
        'Style': 'Points',
        'Topic': _with_filter(_SENSOR_QOS) | {'Value': f'/{ns}/scan'},
        'Use Fixed Frame': True, 'Use rainbow': True, 'Value': True,
    }


def _odometry(ns, colour):
    return {
        'Angle Tolerance': 0.1, 'Class': 'rviz_default_plugins/Odometry',
        'Covariance': {
            'Orientation': {'Alpha': 0.5, 'Color': '255; 255; 127',
                            'Color Style': 'Unique', 'Frame': 'Local',
                            'Offset': 1, 'Scale': 1, 'Value': True},
            'Position': {'Alpha': 0.3, 'Color': '204; 51; 204', 'Scale': 1,
                         'Value': True},
            'Value': False},
        'Enabled': True, 'Keep': 60, 'Name': f'{ns} odometry (EKF)',
        'Position Tolerance': 0.1,
        'Shape': {'Alpha': 0.6, 'Axes Length': 1, 'Axes Radius': 0.1,
                  'Color': colour, 'Head Length': 0.08, 'Head Radius': 0.06,
                  'Shaft Length': 0.15, 'Shaft Radius': 0.03,
                  'Value': 'Arrow'},
        'Topic': _with_filter(_RELIABLE_QOS) | {'Value': f'/{ns}/odometry/filtered'},
        'Value': True,
    }


def _particles(ns, colour):
    return {
        'Alpha': 0.5, 'Arrow Length': 0.2, 'Axes Length': 0.3,
        'Axes Radius': 0.01, 'Class': 'rviz_default_plugins/PoseArray',
        'Color': colour, 'Enabled': True, 'Head Length': 0.07,
        'Head Radius': 0.03, 'Name': f'{ns} AMCL particles',
        'Shaft Length': 0.23, 'Shaft Radius': 0.01, 'Shape': 'Arrow (Flat)',
        'Topic': _with_filter(_SENSOR_QOS) | {'Value': f'/{ns}/particle_cloud'},
        'Value': True,
    }


def _path(name, topic, colour, width):
    return {
        'Alpha': 1, 'Buffer Length': 1, 'Class': 'rviz_default_plugins/Path',
        'Color': colour, 'Enabled': True, 'Head Diameter': 0.3,
        'Head Length': 0.2, 'Length': 0.3, 'Line Style': 'Lines',
        'Line Width': width, 'Name': name,
        'Offset': {'X': 0, 'Y': 0, 'Z': 0}, 'Pose Color': '255; 85; 255',
        'Pose Style': 'None', 'Radius': 0.03, 'Shaft Diameter': 0.1,
        'Shaft Length': 0.1,
        'Topic': _with_filter(_RELIABLE_QOS) | {'Value': topic},
        'Value': True,
    }


def _footprint(ns, colour):
    return {
        'Alpha': 1, 'Class': 'rviz_default_plugins/Polygon', 'Color': colour,
        'Enabled': True, 'Name': f'{ns} footprint',
        'Topic': _with_filter(_RELIABLE_QOS) | {
            'Value': f'/{ns}/local_costmap/published_footprint'},
        'Value': True,
    }


def _image(name, topic, enabled):
    return {
        'Class': 'rviz_default_plugins/Image', 'Enabled': enabled,
        'Max Value': 1, 'Median window': 5, 'Min Value': 0, 'Name': name,
        'Normalize Range': True,
        'Topic': dict(_SENSOR_QOS, **{'Value': topic}), 'Value': enabled,
    }


def _cloud(name, topic, enabled):
    return {
        'Alpha': 1, 'Autocompute Intensity Bounds': True,
        'Autocompute Value Bounds': {'Max Value': 10, 'Min Value': -10,
                                     'Value': True},
        'Axis': 'Z', 'Channel Name': 'intensity',
        'Class': 'rviz_default_plugins/PointCloud2', 'Color': '255; 255; 255',
        'Color Transformer': 'RGB8', 'Decay Time': 0, 'Enabled': enabled,
        'Invert Rainbow': False, 'Max Color': '255; 255; 255',
        'Max Intensity': 4096, 'Min Color': '0; 0; 0', 'Min Intensity': 0,
        'Name': name, 'Position Transformer': 'XYZ', 'Selectable': True,
        'Size (Pixels)': 3, 'Size (m)': 0.01, 'Style': 'Points',
        'Topic': _with_filter(_SENSOR_QOS) | {'Value': topic},
        'Use Fixed Frame': True, 'Use rainbow': True, 'Value': enabled,
    }


def _robot_group(idx, ns, arm, nav):
    """Everything belonging to one robot, in one collapsible group."""
    colour = COLOURS[idx % len(COLOURS)]
    displays = [_robot_model(ns), _laser(ns, colour)]

    if nav:
        displays += [
            _odometry(ns, colour),
            _particles(ns, colour),
            _footprint(ns, colour),
            _path(f'{ns} global plan', f'/{ns}/plan', '0; 255; 0', 0.04),
            _path(f'{ns} local plan', f'/{ns}/local_plan', '255; 0; 255', 0.03),
            # The local costmap is a small window that follows the robot, so
            # several of them coexist happily. The global one covers the whole
            # building, so two of them are two opaque sheets stacked on the map
            # and on each other - shipped off, tick on the one you are
            # debugging.
            _map(f'{ns} local costmap', f'/{ns}/local_costmap/costmap',
                 'costmap', 0.6, True),
            _map(f'{ns} global costmap', f'/{ns}/global_costmap/costmap',
                 'costmap', 0.4, False),
        ]

    if arm:
        # Only the arm robots have their cameras bridged at all - see the
        # bridge comment in hospital_fleet.launch.py, where bridging several
        # robots' pairs put ~600 MB/s through DDS and starved the visual servo.
        # A display here for a topic nobody publishes is not an error, but it
        # is a promise the simulation cannot keep, so r3 and r4 get none.
        displays += [
            _image(f'{ns} front camera', f'/{ns}/front_camera/image', True),
            _image(f'{ns} wrist camera', f'/{ns}/camera/image', False),
            _cloud(f'{ns} front cloud', f'/{ns}/front_camera/points', False),
            _cloud(f'{ns} wrist cloud', f'/{ns}/camera/points', False),
        ]

    role = ', '.join(['nav'] * nav + ['arm'] * arm) or 'parked'
    return {'Class': 'rviz_common/Group', 'Displays': displays,
            'Enabled': True, 'Name': f'{ns} ({role})'}


def _tool_topic(topic):
    return dict(_RELIABLE_QOS, **{'Value': topic})


def fleet_rviz_config(robots, arm_robots=(), nav_robots=(),
                      tool_ns=None, view_centre=(0.0, 0.0)):
    """Write an RViz config for `robots` and return its path.

    `robots` is fleet_layout.ROBOTS: (namespace, x, y, yaw) tuples.
    `arm_robots` and `nav_robots` are fleet_layout's lists of the same names, which
    decide which displays a robot gets - a parked robot has no costmaps and no
    cameras to show.

    THE 2D TOOLS CAN ONLY EVER ADDRESS ONE ROBOT. "2D Pose Estimate" and "2D
    Goal Pose" publish to a single topic each, and a fleet has one per robot, so
    they are wired to `tool_ns` (the first navigating robot by default). To
    drive another robot from RViz, edit the topic in the Tool Properties panel -
    /r2/goal_pose instead of /r1/goal_pose - rather than expecting the click to
    reach whichever robot you clicked nearest.
    """
    names = [ns for ns, *_ in robots]
    tool_ns = tool_ns or (list(nav_robots)[0] if nav_robots else names[0])

    displays = [
        _grid(),
        # THE JOB ITSELF, as the task manager sees it: the four tables, which
        # delivery slot is taken and by whom, which parking vertex is claimed,
        # and a status board over the lobby. None of this exists as a topic any
        # other display could show -- it lives in the manager's head, so the
        # manager draws it.
        {'Class': 'rviz_default_plugins/MarkerArray', 'Enabled': True,
         'Name': 'Task manager', 'Namespaces': {}, 'Queue Size': 100,
         'Topic': dict(_with_filter(_RELIABLE_QOS), Value='/fleet/markers'),
         'Value': True},
        # THE ONE THING THE FLEET SHARES. Every robot has its own odom island
        # and its own everything else; `map` is the single frame that makes
        # their poses comparable, which is why the fixed frame below is `map`
        # and not any robot's own frame.
        _map('Map (shared)', '/map', 'map', 0.7, True),
        _tf(False),
    ]
    displays += [
        _robot_group(idx, ns, ns in arm_robots, ns in nav_robots)
        for idx, (ns, *_) in enumerate(robots)
    ]

    config = {
        'Panels': [
            {'Class': 'rviz_common/Displays', 'Help Height': 78,
             'Name': 'Displays',
             'Property Tree Widget': {
                 'Expanded': ['/Global Options1'] +
                             [f'/{ns}1' for ns in names[:2]],
                 'Splitter Ratio': 0.5},
             'Tree Height': 620},
            {'Class': 'rviz_common/Selection', 'Name': 'Selection'},
            {'Class': 'rviz_common/Tool Properties',
             'Expanded': ['/2D Goal Pose1'], 'Name': 'Tool Properties',
             'Splitter Ratio': 0.58},
            {'Class': 'rviz_common/Views', 'Expanded': ['/Current View1'],
             'Name': 'Views', 'Splitter Ratio': 0.5},
            {'Class': 'rviz_common/Time', 'Experimental': False,
             'Name': 'Time', 'SyncMode': 0,
             'SyncSource': f'{names[0]} LIDAR'},
        ],
        'Visualization Manager': {
            'Class': '',
            'Displays': displays,
            'Enabled': True,
            'Global Options': {'Background Color': '48; 48; 48',
                               'Fixed Frame': 'map', 'Frame Rate': 30},
            'Name': 'root',
            'Tools': [
                {'Class': 'rviz_default_plugins/Interact',
                 'Hide Inactive Objects': True},
                {'Class': 'rviz_default_plugins/MoveCamera'},
                {'Class': 'rviz_default_plugins/Select'},
                {'Class': 'rviz_default_plugins/FocusCamera'},
                {'Class': 'rviz_default_plugins/Measure', 'Line color': '128; 128; 0'},
                {'Class': 'rviz_default_plugins/SetInitialPose',
                 'Covariance x': 0.25, 'Covariance y': 0.25,
                 'Covariance yaw': 0.068,
                 'Topic': _tool_topic(f'/{tool_ns}/initialpose')},
                {'Class': 'rviz_default_plugins/SetGoal',
                 'Topic': _tool_topic(f'/{tool_ns}/goal_pose')},
                {'Class': 'rviz_default_plugins/PublishPoint',
                 'Single click': True,
                 'Topic': _tool_topic('/clicked_point')},
            ],
            'Transformation': {'Current': {'Class': 'rviz_default_plugins/TF'}},
            'Value': True,
            'Views': {
                # Looking straight down on the ring from far enough up to hold
                # all four robots at once, which is the view the fleet is for.
                'Current': {
                    'Class': 'rviz_default_plugins/Orbit', 'Distance': 26.0,
                    'Enable Stereo Rendering': {
                        'Stereo Eye Separation': 0.06,
                        'Stereo Focal Distance': 1, 'Swap Stereo Eyes': False,
                        'Value': False},
                    'Focal Point': {'X': float(view_centre[0]),
                                    'Y': float(view_centre[1]), 'Z': 0.0},
                    'Focal Shape Fixed Size': True, 'Focal Shape Size': 0.05,
                    'Invert Z Axis': False, 'Name': 'Current View',
                    'Near Clip Distance': 0.01, 'Pitch': 1.2,
                    'Target Frame': 'map',
                    'Value': 'Orbit (rviz_default_plugins)', 'Yaw': 3.14},
                'Saved': [
                    {'Class': 'rviz_default_plugins/Orbit', 'Distance': 6.0,
                     'Enable Stereo Rendering': {
                         'Stereo Eye Separation': 0.06,
                         'Stereo Focal Distance': 1,
                         'Swap Stereo Eyes': False, 'Value': False},
                     'Focal Point': {'X': 0, 'Y': 0, 'Z': 0},
                     'Focal Shape Fixed Size': True, 'Focal Shape Size': 0.05,
                     'Invert Z Axis': False, 'Name': f'Chase {ns}',
                     'Near Clip Distance': 0.01, 'Pitch': 0.6,
                     'Target Frame': f'{ns}/base_link',
                     'Value': 'Orbit (rviz_default_plugins)', 'Yaw': 3.1}
                    for ns in names
                ],
            },
        },
        # No QMainWindow State: it encodes dock sizes for one exact set of
        # panels, and this set changes with the fleet. RViz lays itself out from
        # scratch without it.
        'Window Geometry': {
            'Displays': {'collapsed': False},
            'Height': 1000, 'Hide Left Dock': False, 'Hide Right Dock': True,
            'Selection': {'collapsed': False},
            'Time': {'collapsed': False},
            'Tool Properties': {'collapsed': False},
            'Views': {'collapsed': True},
            'Width': 1800, 'X': 60, 'Y': 30,
        },
    }

    out = os.path.join(tempfile.mkdtemp(prefix='fleet_rviz_'), 'fleet.rviz')
    with open(out, 'w') as fh:
        yaml.safe_dump(config, fh, default_flow_style=False, sort_keys=True)
    return out
