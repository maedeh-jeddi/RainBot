import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from pickplace_arm_bringup.fleet_rviz import fleet_rviz_config


def _launch_module(bringup_share, filename):
    """Load another launch file as a module, for the constants at its top.

    Same loader, same reason, as mission_hospital_aws_rviz.launch.py: ROBOTS,
    ARM_ROBOTS and NAV_ROBOTS live in aws_hospital_fleet.launch.py and are the
    fleet's single source of truth, so the RViz config has to be generated from
    them or a fifth robot appears in Gazebo and not in RViz. Launch files are
    installed to share/ rather than onto the Python path, so they are loaded by
    path. Importing one only defines constants and functions;
    generate_launch_description is not called here.
    """
    path = os.path.join(bringup_share, 'launch', filename)
    spec = importlib.util.spec_from_file_location(filename.split('.')[0], path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_launch_description():
    """The relay, with RViz watching both robots at once.

    mission_hospital_relay.launch.py plus an RViz session configured for the
    fleet. Nothing about the simulation changes; if it runs without RViz it runs
    with it.

    THIS IS THE RUN RVIZ WAS GENERATED FOR. fleet_rviz.py draws every robot in
    its own colour - its model, LIDAR, particle cloud, plans and costmaps - and
    a single robot doing a whole route never needed that. A relay does: the
    interesting moment is two robots facing each other 1.55 m apart in a
    corridor, one setting a rack down and the other picking it up, and the only
    view that shows both of them and the map they share is this one.

    The 2D tools go to the CARRIER, because it is the robot whose goals you
    would want to override by hand - it drives the long leg. To drive the
    receiver from RViz instead, edit the topic in the Tool Properties panel
    (/r2/goal_pose for /r1/goal_pose); see the note in fleet_rviz_config.

    RVIZ IS NOT FREE AND THIS MACHINE IS ALREADY THE BOTTLENECK - and this run
    loads it harder than the single-robot one, because both robots navigate AND
    both carry a move_group for the whole run rather than one of them idling. If
    the real-time factor drops or controllers start failing to come up, run with
    `use_gazebo_gui:=false`: once RViz is up the Gazebo GUI is the redundant
    window, and it is the more expensive of the two.
    """
    bringup_share = get_package_share_directory('pickplace_arm_bringup')
    fleet = _launch_module(bringup_share, 'aws_hospital_fleet.launch.py')
    scenario = _launch_module(bringup_share, 'mission_hospital_relay.launch.py')

    relay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch',
                         'mission_hospital_relay.launch.py')))

    rviz_config = fleet_rviz_config(
        fleet.ROBOTS,
        arm_robots=fleet.ARM_ROBOTS,
        nav_robots=fleet.NAV_ROBOTS,
        tool_ns=scenario.CARRIER_NS,
        view_centre=fleet.RING_CENTRE,
    )

    # The env -u prefix is not decoration: without it rviz2 picks a snap's
    # libpthread up at runtime and dies instantly with a GLIBC_PRIVATE symbol
    # lookup error. Same prefix, same reason, as the other RViz launches.
    #
    # No MoveIt parameters are passed, and so there is no MotionPlanning panel:
    # that panel belongs to ONE move_group, and this run has two of them busy at
    # different times. Watch here; plan in the mission nodes.
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        prefix=('env -u GTK_PATH -u GTK_EXE_PREFIX -u LOCPATH '
                '-u GDK_PIXBUF_MODULE_FILE -u GDK_PIXBUF_MODULEDIR '
                '-u GIO_MODULE_DIR -u GTK_IM_MODULE_FILE'),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_gazebo_gui', default_value='true',
            description='false runs Gazebo headless; RViz still shows '
                        'everything the robots publish.'),
        # HEADLESS is read by aws_hospital_fleet.launch.py with os.environ.get,
        # at the moment its generate_launch_description runs - which is when the
        # include below is EXECUTED, not now. This action therefore has to come
        # before that include in this list, and does.
        SetEnvironmentVariable('HEADLESS', PythonExpression(
            ["'0' if '", LaunchConfiguration('use_gazebo_gui'),
             "' == 'true' else '1'"])),
        rviz,
        relay,
    ])
