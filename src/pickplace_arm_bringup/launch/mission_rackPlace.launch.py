import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from pickplace_arm_bringup.hospital_pickplace_layout import (
    COLUMNS, COLUMN_YAW, RACKS, RACK_SPAWN_Z, SPAWN, TABLE, TABLE_Z)


def generate_launch_description():
    """mission_pickPlace's run, moved into the AWS hospital and carrying racks.

    THIS IS mission_pickPlace.launch.py WITH TWO THINGS CHANGED AND NOTHING
    ELSE. Same three-payload colour sort off a table onto three matching
    columns, same staging, same readiness gates, same Nav2, same MoveIt, same
    RViz. What differs:

      1. THE WORLD is aws_hospital.sdf instead of tugbot_warehouse.sdf -- a real
         furnished building with walls, wards, furniture and standing people. It
         contains NOTHING THAT MOVES: the nine walking actors and the pushed
         wheelchair that world used to carry were taken out, so the only thing
         moving in the building is the robot. Three standing figures remain,
         because a figure at a fixed pose is scenery, not traffic.

      2. THE PAYLOAD is a sample rack per colour instead of a coloured cube.

    ODOMETRY AND LOCALIZATION ARE DELIBERATELY UNTOUCHED. The warehouse run
    localized well, and none of the reasons why change with the building -- same
    chassis, same wheel friction, same IMU, same EKF, same LIDAR, same particle
    filter settings (config/amcl_hospital.yaml is config/amcl_tugbot.yaml with
    the map and the initial pose swapped and nothing else).

    THE MAP IS GENERATED, NOT DRIVEN. maps/aws_hospital.{pgm,yaml} come from
    aws_hospital_map.py, which slices the world's own collision geometry at the
    LIDAR's height. So map frame == world frame exactly, with no SLAM drift
    between them, and AMCL's initial pose is simply the robot's spawn pose.
    Re-run that script whenever the world file changes.

    THE PROPS ARE SPAWNED, NOT PUT IN THE WORLD FILE, and they are deliberately
    absent from the map. The racks are carried away, so a map obstacle where one
    used to be is worse than none at all; the table (0.30 m) and two of the
    three columns sit entirely BELOW the 0.4466 m scan plane, so a LIDAR could
    not return them anyway and stamping them would bias AMCL rather than help
    the planner. Same split tugbot_warehouse uses. See hospital_pickplace_layout
    for where they stand and how that spot was measured.
    """
    desc_share = get_package_share_directory('pickplace_arm_description')
    bringup_share = get_package_share_directory('pickplace_arm_bringup')
    models = os.path.join(desc_share, 'models')
    sim = {'use_sim_time': True}

    # -world is not optional here. ros_gz_sim's `create` asks Gazebo for the
    # list of world names first and gives up on a fixed timeout; this world
    # streams in 175 models with their meshes and does not answer in time. Same
    # reason gazebo.launch.py gates the robot's own spawn.
    def spawn(model, name, x, y, z, yaw=0.0):
        return Node(package='ros_gz_sim', executable='create', output='screen',
                    arguments=['-world', 'aws_hospital',
                               '-file', os.path.join(models, model, 'model.sdf'),
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
            bringup_share, 'maps', 'aws_hospital.yaml')}])
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'amcl_hospital.yaml'), sim])
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[sim, {'autostart': True, 'bond_timeout': 0.0,
                          'node_names': ['map_server', 'amcl']}])
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'nav2.launch.py')))

    # Poses come from hospital_pickplace_layout, which is also what the mission
    # node reads. They must not be duplicated here: a prop spawned somewhere the
    # mission does not expect it is the single easiest way to lose an afternoon
    # in this project, and it has been lost that way before.
    #
    # SPAWN Z IS THE MODEL ORIGIN, and the three models disagree about where
    # theirs is. The table box is centred, so it spawns at half its height. A
    # rack carries its origin on its own tray underside, so it spawns at exactly
    # the height of the table top it stands on. The columns are modelled from
    # their base, so they spawn at 0.
    table = spawn('table', 'table', TABLE[0], TABLE[1], TABLE_Z, TABLE[2])
    racks = [spawn(f'rack_{c}', f'rack_{c}', xy[0], xy[1], RACK_SPAWN_Z)
             for c, xy in RACKS]
    columns = [spawn(f'apriltag_column_{i + 1}', f'apriltag_column_{i + 1}',
                     xy[0], xy[1], 0.0, COLUMN_YAW)
               for i, _h, xy in COLUMNS]

    mission = Node(
        package='pickplace_arm_bringup', executable='mission_rackPlace',
        output='screen', parameters=[sim])

    # ---- readiness gates -----------------------------------------------------
    # Same gates, same reasoning and the same generous timeouts as
    # mission_pickPlace.launch.py: each is a short-lived node that exits 0 once
    # its condition holds (or once its timeout expires, so a stuck check can
    # never brick a launch that would otherwise work), and the stage behind it
    # chains on OnProcessExit. See wait_for.py.
    #
    # THE TIMEOUTS ARE RAISED, AND ONLY THE TIMEOUTS. This world loads 175
    # models with their meshes and runs at a real-time factor near 0.5, so every
    # stage genuinely takes longer in wall-clock terms -- but a gate exits the
    # moment its condition is met, so a longer ceiling costs nothing when the
    # machine is quick and saves the whole run when it is not. A gate that gives
    # up does NOT stop anything: wait_for logs "Continuing anyway" and exits 0,
    # so a short timeout does not fail the run, it starts the next stage too
    # early and the failure surfaces somewhere that looks unrelated.
    def gate(label, timeout, *args):
        return Node(package='pickplace_arm_bringup', executable='wait_for',
                    name=f'wait_for_{label}', output='screen', parameters=[sim],
                    arguments=['--label', label, '--timeout', str(timeout), *args])

    gate_world = gate('world', 300.0, '--clock-stable', '0.5',
                      '--tf', 'odom', 'base_link')
    gate_clock = gate('clock', 300.0, '--clock-stable', '3.0',
                      '--tf', 'odom', 'base_link')
    # LOAD-BEARING: the lifecycle manager must not be started in the same breath
    # as the two nodes it manages. Gazebo is still streaming this world in, the
    # manager loses the race, "Failed to bring up all requested nodes" -- after
    # which nothing ever publishes map->odom, no `map` frame exists at all, and
    # the mission dies on "no map->base_link TF". Waiting for both get_state
    # services to actually exist removes the race rather than hiding it behind a
    # delay.
    gate_lifecycle = gate('lifecycle', 180.0,
                          '--service', '/map_server/get_state',
                          '--service', '/amcl/get_state')
    gate_amcl = gate('amcl', 300.0, '--tf', 'map', 'base_link',
                     '--topic', '/amcl_pose')
    gate_nav2 = gate('nav2', 300.0, '--action', '/navigate_to_pose',
                     '--action', '/move_action')

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
        # Arguments MUST be declared before anything that substitutes them:
        # launch executes this list in order, so a SetEnvironmentVariable
        # referencing an as-yet-undeclared LaunchConfiguration aborts the whole
        # launch with "launch configuration ... does not exist".
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true'),
        SetEnvironmentVariable('WORLD', 'aws_hospital.sdf'),
        # THE WORLD ORIGIN IS INSIDE A WALL, so unlike the warehouse this world
        # cannot spawn at (0, 0). These three are the same pose AMCL is seeded
        # with in amcl_hospital.yaml and the origin hospital_pickplace_layout
        # measures everything from; they move together or not at all.
        SetEnvironmentVariable('SPAWN_X', str(SPAWN[0])),
        SetEnvironmentVariable('SPAWN_Y', str(SPAWN[1])),
        SetEnvironmentVariable('SPAWN_YAW', str(SPAWN[2])),
        # The GUI needs longer to build its scene than the server needs to be
        # ready, and a model inserted while it is still streaming is dropped:
        # it lands in the ECM, physics and the sensors see it, `gz model --list`
        # reports it, and it is simply never drawn. Hold the robot's insert
        # until the viewport has settled. See gazebo.launch.py's spawn gate.
        SetEnvironmentVariable('SPAWN_DELAY', '45'),
        SetEnvironmentVariable('HEADLESS', PythonExpression(
            ["'0' if '", LaunchConfiguration('use_gazebo_gui'),
             "' == 'true' else '1'"])),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.join(bringup_share, 'config', 'fastdds_udp_only.xml')),
        gazebo,
        move_group,
        # RViz starts immediately rather than on a timer: it tolerates topics
        # that do not exist yet, so there is nothing to wait for, and starting
        # it first means the world appears as it loads.
        rviz,
        gate_world,
        RegisterEventHandler(OnProcessExit(
            target_action=gate_world, on_exit=[table] + columns)),
        # Racks chain off the TABLE's spawner exiting, not off a timer: they are
        # placed at the table top's height, so spawning them before the table
        # exists drops all three on the floor.
        RegisterEventHandler(OnProcessExit(target_action=table, on_exit=racks)),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_world, on_exit=[gate_clock])),
        # map_server and amcl first, WITHOUT their lifecycle manager -- it only
        # follows once gate_lifecycle has seen both answer (see above).
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
