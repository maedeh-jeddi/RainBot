import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from pickplace_arm_bringup.fleet_layout import ARM_ROBOTS, ROBOTS, WORLD_ENTITY
from pickplace_arm_bringup.rack_table_layout import (
    RACK_SPAWN_Z, STATIC_TABLES, TABLE_SPAWN_Z, collection_points,
    delivery_table_pose,
)


def generate_launch_description():
    """The fleet plus the props it works with: four tables and three racks.

    STILL NO MISSION. This is hospital_fleet.launch.py unchanged -- three robots
    in front of reception, each localised and navigable -- with the furniture the
    job needs added to it: three collection tables, each carrying one coloured
    sample rack, and one delivery table with three slots in a row. Nobody is told
    to do anything with them yet; assigning and driving the errands is the task
    manager's job and lives in its own step.

    WHY A SEPARATE LAUNCH FILE. hospital_fleet.launch.py is infrastructure -- it
    describes robots, not work -- and keeping it that way means it can be brought
    up on its own to debug the fleet without any props in the world. This file is
    the one that says what the robots are FOR, and it is where the task manager
    will be added.

    THE PROPS ARE SPAWNED, NOT WRITTEN INTO aws_hospital.sdf. The world file is
    the building; a rack that gets carried away is payload. The tables are a
    middle case -- they never move, so they DO belong in the map, and
    aws_hospital_map.py stamps them from the same rack_table_layout this file
    spawns them from. Racks are deliberately absent from the map: an obstacle
    where a prop no longer is, is worse than one that was never drawn.

    SPAWN Z IS THE MODEL ORIGIN, and the two models disagree about where theirs
    is. rack_table carries its origin at its FEET, so it spawns at 0 and its top
    lands at 0.3238. A rack carries its origin on its own tray underside, so it
    spawns at exactly the height of the top it stands on.

    THE STARTUP DETACH IS LOAD-BEARING, AND IT HAS TO REACH EVERY ROBOT. The
    DetachableJoint plugin creates its joint the instant the child model appears,
    without anybody asking -- so the moment rack_red spawns it is welded to EVERY
    robot carrying that plugin, wherever they happen to be standing. With one
    robot that cost a single detach at startup; with three robots and three racks
    it is nine, and skipping any one of them tows that rack off its table
    sideways as soon as that robot moves. There is no mission node here to
    publish them, so this file does.
    """
    bringup_share = get_package_share_directory('pickplace_arm_bringup')
    desc_share = get_package_share_directory('pickplace_arm_description')
    models = os.path.join(desc_share, 'models')

    fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'hospital_fleet.launch.py')))

    # -world is not optional. ros_gz_sim's `create` asks Gazebo for the list of
    # world names first and gives up on a fixed timeout; this world streams in
    # 175 models with their meshes and does not answer in time. Same reason
    # gazebo.launch.py gates the robots' own spawns.
    def spawn(model, name, x, y, z, yaw=0.0):
        return Node(package='ros_gz_sim', executable='create', output='screen',
                    arguments=['-world', WORLD_ENTITY,
                               '-file', os.path.join(models, model, 'model.sdf'),
                               '-name', name, '-x', str(x), '-y', str(y),
                               '-z', str(z), '-Y', str(yaw)])

    # EVERYTHING HERE WAITS FOR THE WHOLE FLEET, and both halves of that matter.
    #
    # The GZ SERVER has to exist at all: `create` calls /world/<name>/create, and
    # at t=0 Gazebo is still streaming 175 models in and cannot service it, so an
    # ungated spawner dies of a timeout and the prop is simply never there.
    #
    # THE ROBOTS have to exist too, and that is the less obvious one. The
    # DetachableJoint plugin lives on the ROBOT and welds itself to any matching
    # child model that is already present when the robot appears. Spawn the racks
    # first and detach before the robots arrive, and every one of those detaches
    # is addressed at nothing: the robots then spawn, weld all three racks on
    # sight, and nothing ever breaks those joints. The first robot to move drags
    # all three racks off their tables.
    #
    # So: fleet up -> tables -> racks -> detaches, strictly in that order.
    # Waiting on the LAST robot's drive controller publishing odometry is the
    # same condition hospital_fleet.launch.py uses for the same purpose -- it
    # means every robot has spawned and every controller_manager has finished.
    last_ns = ROBOTS[-1][0]
    prop_gate = Node(
        package='pickplace_arm_bringup', executable='wait_for',
        name='wait_for_fleet_props', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--label', 'fleet props', '--timeout', '900',
                   '--clock-stable', '0.5',
                   '--topic', f'/{last_ns}/diff_drive_controller/odom'])

    # Tables first. Poses come from rack_table_layout, which is also what the map
    # generator stamps -- they must not be duplicated here, because a table
    # spawned somewhere the map does not have one is exactly the bug that has
    # already wedged a robot against thin air twice in this project.
    tables = []
    dx, dy, dyaw = delivery_table_pose()
    tables.append(spawn('rack_table', 'table_delivery', dx, dy, TABLE_SPAWN_Z, dyaw))
    for name, _colour, (tx, ty, tyaw), _rack, _stand in collection_points():
        tables.append(spawn('rack_table', f'table_{name}', tx, ty,
                            TABLE_SPAWN_Z, tyaw))
    assert len(tables) == len(STATIC_TABLES)

    # Racks chain off their OWN table's spawner exiting, not off a timer: each is
    # placed at its table's top height, so spawning one before its table exists
    # drops it on the floor.
    racks, rack_chain = [], []
    for idx, (name, colour, _table, (rx, ry), _stand) in enumerate(
            collection_points(), start=1):
        rack = spawn(f'rack_{colour}', f'rack_{colour}', rx, ry, RACK_SPAWN_Z)
        racks.append(rack)
        rack_chain.append(RegisterEventHandler(
            OnProcessExit(target_action=tables[idx], on_exit=[rack])))

    # Nine welds to break: one per (robot, rack) pair. See rack_release.py for
    # why this is a node that publishes REPEATEDLY rather than nine
    # `ros2 topic pub --once` calls -- in short, those nine all reported
    # publishing successfully and not one weld broke, because `--once` tears the
    # publisher down before the sample is delivered and nine of them starting at
    # the same instant is exactly when that bites. The cost of getting it wrong
    # is not subtle: every rack is dragged off its table and the robots wedge
    # themselves on the contacts.
    release = Node(
        package='pickplace_arm_bringup', executable='rack_release',
        name='rack_release', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[a for ns in ARM_ROBOTS for a in ('--robot', ns)]
        + [a for _n, colour, _t, _r, _s in collection_points()
           for a in ('--model', f'rack_{colour}')]
        + ['--duration', '20', '--rate', '2'])
    detaches = [release]

    # --- the job -------------------------------------------------------------
    #
    # One mission node per robot, in that robot's namespace, plus one manager in
    # the global namespace. The split is deliberate: a mission node knows how to
    # run ONE errand and nothing about the others; the manager is the only thing
    # that can see all three at once, so it is where assignment, the delivery
    # slot row, the parking vertices and the anti-collision sequencing live.
    #
    # HELD BACK BEHIND THE SAME GATE AS THE PROPS AND THEN SOME. A mission node
    # constructs a MoveIt2 client and a pile of action clients; started before
    # its robot's Nav2 stack is serving, it would sit retrying while the machine
    # is at its busiest.
    #
    # THIS GATE IS DELIBERATELY COARSE, and the TASK MANAGER makes the fine
    # decision. An action server appearing is not proof that a robot is ready:
    # nav_bringup RESETs and retries a stalled stack, so during a retry storm
    # navigate_to_pose comes and goes, and a gate that samples once will happily
    # release the fleet in one of those windows. The manager waits for each
    # robot's localization to hold steady before giving it an errand -- see
    # wait_ready() in task_manager.py.
    mission_gate = Node(
        package='pickplace_arm_bringup', executable='wait_for',
        name='wait_for_mission', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--label', 'mission', '--timeout', '900']
        # NO TF CHECK HERE, DELIBERATELY. The condition that matters is that
        # each robot's AMCL is localizing, and the obvious way to test it is
        # map -> <ns>/base_link. But a tf2 listener in PYTHON deserialises every
        # /tf message, and three robots publish full TF trees at 50 Hz: measured,
        # this one gate cost 24% of a core for the whole bring-up, in exactly
        # the window where nav2_lifecycle_manager's hardcoded 2 s deadlines are
        # being missed. It was buying a check the mission node already makes for
        # itself -- wait_for_localization() waits for that same transform -- so
        # the gate stays cheap and the MISSION NODE is patient instead.
        + [a for ns in ARM_ROBOTS
           for a in ('--action', f'/{ns}/navigate_to_pose')]
        + [a for ns in ARM_ROBOTS
           for a in ('--action', f'/{ns}/move_action')])

    missions = [
        Node(package='pickplace_arm_bringup', executable='mission_delivery',
             namespace=ns, name='mission_delivery', output='screen',
             parameters=[{'use_sim_time': True}])
        for ns in ARM_ROBOTS
    ]
    manager = Node(
        package='pickplace_arm_bringup', executable='task_manager',
        name='task_manager', output='screen',
        parameters=[{'use_sim_time': True}])

    # Hung off the LAST RACK's spawner, not the last table's. The racks are
    # created here rather than inside the event handlers precisely so that they
    # stay addressable as launch actions and this ordering can be stated exactly:
    # every rack exists before any detach is published. Chaining off a table
    # instead would race its own rack, and a detach addressed at a model that
    # does not exist yet is consumed silently -- the weld is then made
    # afterwards and never broken, which looks like the rack being towed off its
    # table by a robot that never touched it.
    detach_after_racks = RegisterEventHandler(
        OnProcessExit(target_action=racks[-1], on_exit=detaches))

    return LaunchDescription([
        fleet,
        prop_gate,
        # Registered BEFORE anything they watch can exit.
        *rack_chain,
        detach_after_racks,
        RegisterEventHandler(OnProcessExit(target_action=prop_gate,
                                           on_exit=tables)),
        # The mission gate starts once the props are down -- the racks must
        # exist and be released before any robot is told to go and fetch one.
        RegisterEventHandler(OnProcessExit(target_action=racks[-1],
                                           on_exit=[mission_gate])),
        RegisterEventHandler(OnProcessExit(target_action=mission_gate,
                                           on_exit=missions + [manager])),
    ])
