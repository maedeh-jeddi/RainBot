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


def generate_launch_description():
    """Mission 2 in the downloaded Tugbot-warehouse world (OpenRobotics
    "Tugbot in Warehouse" Fuel world): sort 3 coloured boxes off a table onto
    3 matching-coloured columns, then park -- same behaviour and layout as
    mission_2.launch.py (this world's open aisle is clear at those exact
    coordinates), just pointed at a different world + map + AMCL config, and
    running the colour-detection-gated Mission2Tugbot instead of Mission2.

    Map = maps/tugbot_warehouse.yaml, built by SLAM with the robot starting
    at its spawn pose, so map frame == world frame == the plain warehouse
    layout's coordinates unchanged. AMCL seeded from amcl_tugbot.yaml
    (initial pose 0,0,0, matching spawn).
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
            bringup_share, 'maps', 'tugbot_warehouse.yaml')}])
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

    # Same relative layout as the plain warehouse mission -- table 2.30 m
    # ahead of spawn, columns 1.0 m the other way -- this room's open aisle
    # is clear at those coordinates (SLAM map verified).
    #
    # Everything was scaled up for the Husky A200 + FR3. The old props were
    # sized for a 0.38 m-long chassis with a 0.66 m-reach arm whose gripper
    # worked 0.05 m off the floor; the Husky is 0.99 m long and the FR3's base
    # alone sits 0.384 m up, so the old 0.10 m table and 0.08-0.16 m columns
    # were entirely below the front camera's horizon (0.223 m, and 0.332 m when
    # these props were scaled) and could not be seen at approach range.
    #   table   0.18x0.60x0.10  ->  0.25x1.00x0.30   (top at z=0.30)
    #   boxes   0.045 cube      ->  0.06 cube        (still well inside the
    #                                                 Franka Hand's 0.08 m jaw)
    #   columns 0.12sq, 8/12/16 ->  0.20sq, 30/40/50 cm
    # Spawn z is the model ORIGIN: the table box is centred, so z = height/2;
    # a box sits on the table top at 0.30 + 0.06/2; columns are modelled from
    # their base so they spawn at z=0.
    table = spawn('table', 'table', 2.30, 0.0, 0.15)
    boxes = [spawn('box_red', 'box_red', 2.30, -0.22, 0.33),
             spawn('box_green', 'box_green', 2.30, 0.0, 0.33),
             spawn('box_blue', 'box_blue', 2.30, 0.22, 0.33)]
    # Column y-spacing widened 0.30 -> 0.45 to match the wider (0.20) columns
    # and the much wider Husky, so the base can square up on one without its
    # bumper overlapping a neighbour.
    columns = [spawn('apriltag_column_1', 'apriltag_column_1', -1.0, -0.45, 0.0, 3.14159),
               spawn('apriltag_column_2', 'apriltag_column_2', -1.0, 0.0, 0.0, 3.14159),
               spawn('apriltag_column_3', 'apriltag_column_3', -1.0, 0.45, 0.0, 3.14159)]

    mission = Node(
        package='pickplace_arm_bringup', executable='mission_pickPlace',
        output='screen', parameters=[sim])

    # ---- readiness gates -----------------------------------------------------
    # These replace the fixed 12/17/75/95/120 s TimerActions this file used to
    # stage itself on. See wait_for.py for the measurements behind the change:
    # in short, the /clock jump-backs that the 75 s localization delay existed
    # to outlast come from an ORPHANED gz server left over from a previous run,
    # not from this world loading. Measured on a clean process table, /clock is
    # up at t+3.2 s, TF odom->base_link at t+5.3 s, and there are zero
    # jump-backs -- so the old schedule idled ~115 s for nothing, while on a
    # dirty table it silently masked the process leak instead of surfacing it.
    #
    # Each gate is a short-lived node that exits 0 once its condition holds
    # (or once its timeout expires, so a stuck check can never brick a launch
    # that would otherwise work), and the stage behind it is chained on
    # OnProcessExit. The generous timeouts are what preserve the old
    # worst-case behaviour: a genuinely slow or contended machine still ends
    # up starting the same things in the same order.
    def gate(label, timeout, *args):
        return Node(package='pickplace_arm_bringup', executable='wait_for',
                    name=f'wait_for_{label}', output='screen', parameters=[sim],
                    arguments=['--label', label, '--timeout', str(timeout), *args])

    # The world is up and simulating: props can be dropped into it.
    gate_world = gate('world', 120.0, '--clock-stable', '0.5',
                      '--tf', 'odom', 'base_link')
    # The clock has been strictly monotonic for 3 s. This is the one AMCL
    # actually cares about -- bringing it up mid-jump is what used to SIGABRT
    # it -- so this gate, not a wall-clock delay, is what guards localization.
    gate_clock = gate('clock', 150.0, '--clock-stable', '3.0',
                      '--tf', 'odom', 'base_link')
    # map_server and amcl are up far enough to answer lifecycle calls. THIS ONE
    # IS LOAD-BEARING, and it is the one thing the faster startup broke: the
    # lifecycle manager used to be started in the same breath as the two nodes
    # it manages, which was survivable at the old 75 s mark when the machine was
    # already idle, but at t+5 s Gazebo is still streaming the world in and the
    # manager loses the race outright --
    #     Configuring map_server
    #     Failed to change state for node: map_server. Exception:
    #       map_server/get_state service client: async_send_request failed.
    #     Failed to bring up all requested nodes. Aborting bringup.
    # after which nothing ever publishes map->odom, no `map` frame exists at
    # all, and the mission dies on "no map->base_link TF". Waiting for the two
    # get_state services to actually exist removes the race rather than
    # re-hiding it behind a delay.
    gate_lifecycle = gate('lifecycle', 90.0,
                          '--service', '/map_server/get_state',
                          '--service', '/amcl/get_state')
    # AMCL is not merely alive but ACTIVE and localizing: map->base_link is the
    # transform nav2's costmaps and the mission itself both require, and it only
    # exists once amcl has been activated and has fused a scan. Waiting on the
    # TF rather than on /amcl_pose alone is what makes this gate mean
    # "localization works" instead of "a node is running".
    gate_amcl = gate('amcl', 120.0, '--tf', 'map', 'base_link',
                     '--topic', '/amcl_pose')
    # Both action servers the mission drives are actually accepting goals.
    # Waiting on these is what makes the mission start EARLY on a fast machine
    # and still-correctly on a slow one.
    gate_nav2 = gate('nav2', 120.0, '--action', '/navigate_to_pose',
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
        #
        # RViz is on by default: it is the stack's actual instrument panel
        # (map, costmaps, plans, both camera feeds, the MoveIt planning scene),
        # and with the Gazebo GUI also up the two together are the heaviest
        # thing this launch does to a 16 GB box. use_gazebo_gui:=false drops
        # the Gazebo window and leaves RViz as the single viewer -- sensor
        # rendering happens on the gz SERVER, so nothing is lost but scenery.
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true'),
        SetEnvironmentVariable('WORLD', 'tugbot_warehouse.sdf'),
        # gazebo.launch.py takes its headless decision from this env var, and
        # reads it when the include below is evaluated -- which happens after
        # this action runs, so setting it here is enough (same pattern as
        # WORLD above).
        SetEnvironmentVariable('HEADLESS', PythonExpression(
            ["'0' if '", LaunchConfiguration('use_gazebo_gui'),
             "' == 'true' else '1'"])),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.join(bringup_share, 'config', 'fastdds_udp_only.xml')),
        gazebo,
        move_group,
        # RViz starts immediately rather than on a 6 s timer: it tolerates
        # topics that do not exist yet, so there is nothing to wait for, and
        # starting it first means the world appears as it loads.
        rviz,
        gate_world,
        RegisterEventHandler(OnProcessExit(
            target_action=gate_world, on_exit=[table] + columns)),
        # Boxes chain off the TABLE's spawner exiting, not off a timer: they
        # are placed at z=0.33, i.e. resting on the table top, so spawning them
        # before the table exists drops all three on the floor.
        RegisterEventHandler(OnProcessExit(target_action=table, on_exit=boxes)),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_world,
            on_exit=[gate_clock])),
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
