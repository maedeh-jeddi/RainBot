import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Blood sample transport in the hospital world: lab bench -> ward bench.

    Same shape as mission_pickPlace.launch.py - Gazebo, MoveIt, props,
    map_server + AMCL, Nav2, then the mission behind a chain of readiness gates
    - with three differences:

      * the world is hospital_lab.sdf and the robot spawns at world (-6.5, 0),
        which is inside the lab; the world origin sits in the lab's north wall,
        so SPAWN_X/SPAWN_Y are not optional here the way they are in a world
        whose origin happens to be clear.
      * the map is maps/hospital_lab.yaml.
      * the props are one sample_rack and one delivery_dock rather than a table,
        three boxes and three columns.
    """
    desc_share = get_package_share_directory('pickplace_arm_description')
    bringup_share = get_package_share_directory('pickplace_arm_bringup')
    models = os.path.join(desc_share, 'models')
    sim = {'use_sim_time': True}

    def spawn(model, name, x, y, z, yaw=0.0):
        return Node(package='ros_gz_sim', executable='create', output='screen',
                    arguments=['-file', os.path.join(models, model, 'model.sdf'),
                               '-name', name, '-x', str(x), '-y', str(y),
                               '-z', str(z), '-Y', str(yaw)])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'gazebo.launch.py')))

    moveit_config = MoveItConfigsBuilder(
        'pickplace_arm', package_name='pickplace_arm_moveit_config').to_moveit_configs()
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[moveit_config.to_dict(), sim,
                    {'trajectory_execution.allowed_start_tolerance': 0.1}])

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[sim, {'yaml_filename': os.path.join(
            bringup_share, 'maps', 'hospital_lab.yaml')}])
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'amcl_tugbot.yaml'), sim])
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[sim, {'autostart': True, 'bond_timeout': 0.0,
                          'node_names': ['map_server', 'amcl']}])
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'nav2.launch.py')))

    # --- props ---------------------------------------------------------------
    # WORLD coordinates (Gazebo spawns in the world frame); the mission's own
    # constants are in the map frame, which is these shifted by +6.5 in x.
    #
    # Both benches are part of hospital_lab.sdf itself - they are building
    # furniture, not mission payload - so only the rack and the dock are spawned
    # here, exactly as the warehouse mission spawns only its table, boxes and
    # columns into the bare Tugbot world.
    #
    # Spawn z is the model ORIGIN, and both of these models put their origin on
    # their own underside, so each spawns at the height of the shelf it stands
    # on: 0.30 for both. The dock's own top is then 0.38, which is where the
    # rack ends up.
    rack = spawn('sample_rack', 'sample_rack', -5.0, 2.87, 0.30)
    dock = spawn('delivery_dock', 'delivery_dock', 14.0, 0.0, 0.30)

    # Pedestrians are spawned here, NOT placed in the world file, because they
    # move: anything in the world when hospital_map_drive runs is baked into the
    # static map, and a walking person recorded as permanent wall makes the
    # robot dodge where they were rather than where they are. They start at the
    # middle of their beats, in their own lanes, facing along the corridor.
    pedestrians = [
        spawn('pedestrian', 'pedestrian_0', 2.5, 1.10, 0.0),
        spawn('pedestrian', 'pedestrian_1', 7.0, -1.10, 0.0),
    ]
    walk = Node(package='pickplace_arm_bringup', executable='corridor_pedestrians',
                output='screen', parameters=[sim])

    mission = Node(
        package='pickplace_arm_bringup', executable='mission_hospital',
        output='screen', parameters=[sim])

    # --- readiness gates -----------------------------------------------------
    # Identical staging to mission_pickPlace.launch.py; see that file for why
    # each gate exists (in short: they replace fixed sleeps, and the lifecycle
    # one is load-bearing because the manager otherwise races the two nodes it
    # manages while Gazebo is still streaming the world in).
    def gate(label, timeout, *args):
        return Node(package='pickplace_arm_bringup', executable='wait_for',
                    name=f'wait_for_{label}', output='screen', parameters=[sim],
                    arguments=['--label', label, '--timeout', str(timeout), *args])

    gate_world = gate('world', 120.0, '--clock-stable', '0.5',
                      '--tf', 'odom', 'base_link')
    gate_clock = gate('clock', 150.0, '--clock-stable', '3.0',
                      '--tf', 'odom', 'base_link')
    gate_lifecycle = gate('lifecycle', 90.0,
                          '--service', '/map_server/get_state',
                          '--service', '/amcl/get_state')
    gate_amcl = gate('amcl', 120.0, '--tf', 'map', 'base_link',
                     '--topic', '/amcl_pose')
    gate_nav2 = gate('nav2', 120.0, '--action', '/navigate_to_pose',
                     '--action', '/move_action')

    # The env -u prefix is not decoration: without it rviz2 picks a snap's
    # libpthread up at runtime and dies instantly with a GLIBC_PRIVATE symbol
    # lookup error. Same prefix, same reason, as mission_pickPlace.launch.py.
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        prefix=('env -u GTK_PATH -u GTK_EXE_PREFIX -u LOCPATH '
                '-u GDK_PIXBUF_MODULE_FILE -u GDK_PIXBUF_MODULEDIR '
                '-u GIO_MODULE_DIR -u GTK_IM_MODULE_FILE'),
        arguments=['-d', os.path.join(bringup_share, 'config', 'mission.rviz')],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits, sim])

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true'),
        SetEnvironmentVariable('WORLD', 'hospital_lab.sdf'),
        # The hospital world's origin is inside the lab's north wall, so unlike
        # the warehouse this world cannot spawn at (0, 0). These must match the
        # pose the map was built from, or AMCL's (0,0,0) seed is wrong.
        SetEnvironmentVariable('SPAWN_X', '-6.5'),
        SetEnvironmentVariable('SPAWN_Y', '0.0'),
        SetEnvironmentVariable('HEADLESS', PythonExpression(
            ["'0' if '", LaunchConfiguration('use_gazebo_gui'),
             "' == 'true' else '1'"])),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.join(bringup_share, 'config', 'fastdds_udp_only.xml')),
        gazebo,
        move_group,
        rviz,
        gate_world,
        # Props only once the world is simulating, or they drop through a floor
        # that does not exist yet.
        RegisterEventHandler(OnProcessExit(
            target_action=gate_world, on_exit=[rack, dock] + pedestrians)),
        # Start them walking once the last pedestrian has actually spawned,
        # otherwise the first twists are published at models that do not exist.
        RegisterEventHandler(OnProcessExit(
            target_action=pedestrians[-1], on_exit=[walk])),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_world, on_exit=[gate_clock])),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_clock,
            on_exit=[map_server, amcl, gate_lifecycle])),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_lifecycle,
            on_exit=[localization_lifecycle, gate_amcl])),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_amcl, on_exit=[nav2, gate_nav2])),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_nav2, on_exit=[mission])),
    ])
