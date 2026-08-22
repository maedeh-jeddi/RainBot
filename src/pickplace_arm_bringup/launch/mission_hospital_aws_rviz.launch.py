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

    ROBOTS, ARM_ROBOTS and NAV_ROBOTS live in aws_hospital_fleet.launch.py and
    are the fleet's single source of truth - that whole file is generated from
    them, and the RViz config has to be too, or a fifth robot appears in Gazebo
    and not in RViz. MISSION_ROBOT lives in mission_hospital_aws.launch.py for
    the same reason. Launch files are installed to share/ rather than onto the
    Python path, so they are loaded by path. Importing one only defines
    constants and functions; generate_launch_description is not called here.
    """
    path = os.path.join(bringup_share, 'launch', filename)
    spec = importlib.util.spec_from_file_location(filename.split('.')[0], path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_launch_description():
    """The whole thing: four robots, the sample mission, and RViz watching it.

    This is mission_hospital_aws.launch.py - which is itself
    aws_hospital_fleet.launch.py plus the mission props and the mission node -
    with an RViz session on top, configured for the fleet rather than for one
    robot. Nothing about the simulation changes; if it runs without RViz it runs
    with it.

    WHAT RVIZ SHOWS: the shared map, every robot's model in its real pose, every
    robot's LIDAR, and for the two navigating robots their AMCL particle cloud,
    EKF odometry, footprint, local costmap and both plans. The two robots with
    arms also get their camera images. Point clouds, global costmaps and TF are
    present but switched off - see the note on default-enabled displays in
    fleet_rviz.py.

    RVIZ IS NOT FREE AND THIS MACHINE IS ALREADY THE BOTTLENECK. The fleet
    launch file documents four navigating robots plus two move_groups coming to
    ~1200% of 1600% available, and cuts the fleet to two navigating robots for
    exactly that reason. RViz adds a renderer to that, and its own subscriptions
    to sensor topics that are already the most expensive traffic in the system.
    If the real-time factor drops or controllers start failing to come up, run
    with `use_gazebo_gui:=false` - the Gazebo GUI is the redundant window once
    RViz is up, and it is the more expensive of the two.
    """
    bringup_share = get_package_share_directory('pickplace_arm_bringup')
    fleet = _launch_module(bringup_share, 'aws_hospital_fleet.launch.py')
    scenario = _launch_module(bringup_share, 'mission_hospital_aws.launch.py')

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch',
                         'mission_hospital_aws.launch.py')))

    rviz_config = fleet_rviz_config(
        fleet.ROBOTS,
        arm_robots=fleet.ARM_ROBOTS,
        nav_robots=fleet.NAV_ROBOTS,
        # The mission robot is the one whose goals you would want to set by
        # hand, so it gets RViz's 2D Goal Pose and 2D Pose Estimate tools.
        tool_ns=scenario.MISSION_ROBOT,
        view_centre=fleet.RING_CENTRE,
    )

    # The env -u prefix is not decoration: without it rviz2 picks a snap's
    # libpthread up at runtime and dies instantly with a GLIBC_PRIVATE symbol
    # lookup error. Same prefix, same reason, as mission_hospital.launch.py.
    #
    # No MoveIt parameters are passed, and so there is no MotionPlanning panel:
    # that panel belongs to ONE move_group, and this fleet runs two. Plan with
    # the mission node; use RViz to watch.
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
        mission,
    ])
