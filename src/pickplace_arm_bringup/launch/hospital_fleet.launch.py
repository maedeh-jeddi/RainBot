import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
    IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

from pickplace_arm_bringup.fleet_layout import (
    ARM_ROBOTS, FORMATION_CENTRE, NAV_ROBOTS, ROBOTS, SPAWN_Z, WORLD_ENTITY,
)
from pickplace_arm_bringup.fleet_rviz import fleet_rviz_config
from pickplace_arm_bringup.ns_params import diff_drive_frames, namespaced_params

# --- bringing it up faster while developing ----------------------------------
#
# EVERY DELAY BELOW IS THERE BECAUSE SOMETHING FAILED WITHOUT IT, so none of them
# change by default: with no environment set, this file behaves as measured.
# What the overrides buy is iteration speed.
#
#   FLEET_ROBOTS=r1,r2 FLEET_SPAWN_SETTLE=30 FLEET_NAV_SETTLE=0 \
#     ros2 launch pickplace_arm_bringup hospital_fleet.launch.py
_want = os.environ.get('FLEET_ROBOTS')
if _want:
    _keep = [n.strip() for n in _want.split(',') if n.strip()]
    _known = {n for n, *_ in ROBOTS}
    _unknown = [n for n in _keep if n not in _known]
    if _unknown:
        raise RuntimeError(
            f'FLEET_ROBOTS names {_unknown}, not in the fleet {sorted(_known)}')
    ROBOTS = [r for r in ROBOTS if r[0] in _keep]

# HOW LONG TO WAIT BEFORE INSERTING THE FIRST ROBOT, AND HOW FAR APART TO SPACE
# THE REST.
#
# The settle is the GUI gate described at the spawn below: a model inserted while
# the viewport is still streaming 175 models never gets drawn.
#
# The stagger exists for a different reason and is not cosmetic. Every robot's
# controller chain starts when its spawn returns, and all of their
# controller_managers live inside the ONE gz sim process. Spawned together, the
# chains hit that process at the same instant and the losers do not just wait --
# they die: "Could not contact service /r1/controller_manager/list_controllers",
# spawner exits 1, and that robot has no drive controller for the rest of the
# run. Raising the spawner timeout alone did not fix it, because the problem is
# contention rather than a slow answer.
SPAWN_SETTLE = int(os.environ.get('FLEET_SPAWN_SETTLE', 45))
SPAWN_STAGGER = int(os.environ.get('FLEET_SPAWN_STAGGER', 12))
# Seconds between one robot's Nav2 stack and the next, and before the first.
NAV_STAGGER = int(os.environ.get('FLEET_NAV_STAGGER', 25))
# EVERY ROBOT WAITS, INCLUDING THE FIRST. A stagger of idx * NAV_STAGGER gives
# robot zero no delay at all -- and robot zero was the one that kept failing
# while its staggered siblings came up clean, because its controllers finish
# first and its Nav2 bring-up then starts while the others are still being
# inserted into Gazebo and claiming controllers of their own.
NAV_SETTLE = int(os.environ.get('FLEET_NAV_SETTLE', 45))
NAV_NODES = ['controller_server', 'smoother_server', 'planner_server',
             'behavior_server', 'bt_navigator']


def generate_launch_description():
    """Three robots up together in the AWS hospital, in front of reception.

    THE INFRASTRUCTURE STEP, AND NOTHING MORE. Each robot gets its own sensor
    topics, TF tree, controller_manager, EKF, AMCL, Nav2 stack and move_group;
    they share one map_server and one map frame. Nobody is given a task here --
    they come up, they localise, they hold their pose. The world is the static
    hospital, unchanged: nothing in it moves except the robots.

    WHERE THEY STAND is fleet_layout.ROBOTS -- an equilateral triangle in front
    of the reception counter, each robot looking at it. See that module for how
    the formation was measured, and in particular for why it is measured against
    the desk's COLLISION MESH rather than its model origin, which sits 1.3 m
    away from the nearest geometry.

    HOW THE THREE ARE KEPT APART:

      description   robot_ns in pickplace_arm.urdf.xacro prefixes the sensor
                    topics, the sensor frames and the grasp topics, and
                    namespaces the gz_ros2_control plugin
      TF            robot_state_publisher's frame_prefix, matched to those
                    sensor frames; each robot owns an r<N>/odom island
      navigation    ns_params.py rewrites nav2_params.yaml and amcl_hospital.yaml
                    per robot BY VALUE, so `odom` becomes r1/odom while `map`
                    stays shared -- see that module for why a key-based rewrite
                    cannot do this

    WHAT MAKES THEM AVOID EACH OTHER: at this step, nothing explicit. Each
    robot's LIDAR sees the others as obstacles and its local costmap plans
    around them. That is enough for three robots standing still in a building
    this size; it is NOT enough to stop two meeting head-on in a corridor too
    narrow to pass, which is a coordination problem and the subject of its own
    step.

    COST. A full Nav2 stack is five nodes plus AMCL, an EKF, a
    robot_state_publisher and four controller spawners, so the fleet multiplies
    about ten processes -- and Nav2 costs its ~230% of one CPU WHILE THE ROBOT IS
    STANDING STILL, because the costmaps, the behaviour tree and the particle
    filter all run on their own timers whether or not there is a goal.
    move_group is the heaviest single node per robot. Watch the real-time
    factor; if it falls far enough the LIDAR starves and AMCL follows it down.
    """
    desc_share = get_package_share_directory('pickplace_arm_description')
    bringup_share = get_package_share_directory('pickplace_arm_bringup')
    xacro_file = os.path.join(desc_share, 'urdf', 'pickplace_arm.urdf.xacro')
    world_file = os.path.join(desc_share, 'worlds', 'aws_hospital.sdf')
    map_yaml = os.path.join(bringup_share, 'maps', 'aws_hospital.yaml')
    nav2_yaml = os.path.join(bringup_share, 'config', 'nav2_params.yaml')
    amcl_yaml = os.path.join(bringup_share, 'config', 'amcl_hospital.yaml')
    sim = {'use_sim_time': True}

    gz_flags = '-s -r ' if os.environ.get('HEADLESS') == '1' else '-r '
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_flags + world_file,
                          'gz_version': '8'}.items(),
    )

    # ONE map_server for the whole fleet, in the global namespace. The map frame
    # is the only thing the robots share, and it is what makes their separate
    # odom islands comparable -- r1 knowing where r2 is starts here.
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[sim, {'yaml_filename': map_yaml}])
    map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[sim, {'autostart': True, 'node_names': ['map_server']}])

    # ONLY THE FRONT CAMERA IS BRIDGED, AND THE WRIST CAMERA IS NOT.
    #
    # Each robot carries two RGB-D cameras publishing 640x480 organised point
    # clouds. One such cloud is about 5 MB, so one camera is roughly 75 MB/s, and
    # bridging four robots' PAIRS put something like 600 MB/s through
    # parameter_bridge and DDS in the abandoned four-robot attempt. Nothing
    # carries that: the clouds arrive in bursts with long gaps, and the effect
    # lands on the one thing that needs them continuously, the visual servo --
    #
    #   [detect] no point cloud received before timeout      x7 of 12 tries
    #   [claw] lost the box -- aborting approach
    #
    # The wrist camera is pure cost for this plan. It is read by
    # detect_box_pose(), which is only called from PickAndPlace.pick_up_box() --
    # and the mission path used here is claw_pick -> claw_approach -> grab_below,
    # every step of which reads the FRONT camera. Dropping it halves the
    # point-cloud bandwidth at zero functional cost.
    #
    # It still RENDERS on the gz server, which is where the real cost is; turning
    # the sensor off in the description would recover that too, and is the first
    # thing to try if the real-time factor is the problem.
    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    for ns, _, _, _ in ROBOTS:
        bridge_args += [
            f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ]
        if ns in ARM_ROBOTS:
            bridge_args += [
                f'/{ns}/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/{ns}/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                f'/{ns}/front_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ]
            # Grasp control, ROS -> gz (']'), prefixed so one robot's gripper
            # cannot weld a rack to another's hand.
            for colour in ('red', 'green', 'blue'):
                bridge_args += [
                    f'/{ns}/rack_{colour}/attach@std_msgs/msg/Empty]gz.msgs.Empty',
                    f'/{ns}/rack_{colour}/detach@std_msgs/msg/Empty]gz.msgs.Empty',
                ]
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  arguments=bridge_args, output='screen')

    # GATE THE NAVIGATION STACKS ON A CONDITION, NOT ON A CLOCK.
    #
    # These used to start a fixed number of seconds after each robot's
    # controllers came up, and the first robot failed almost every time while its
    # staggered siblings came up clean -- r2 reaching "Managed nodes are active"
    # while r1 sat on "Configuring amcl", or controller_server, or
    # planner_server, a different node each run. The first stack is simply the
    # one that lands while Gazebo is still inserting the other robots.
    #
    # Tuning the delay is a losing game: the world gate measured 63 s early in a
    # session and 104 s hours later on the same machine, so any constant that
    # works now is wrong later. The condition that actually matters is that the
    # LAST robot's LAST controller is publishing -- at which point every robot
    # has spawned and every controller_manager has finished handing out
    # controllers.
    last_ns = ROBOTS[-1][0]
    fleet_gate = Node(
        package='pickplace_arm_bringup', executable='wait_for',
        name='wait_for_fleet', output='screen', parameters=[sim],
        arguments=['--label', 'fleet', '--timeout', '900',
                   '--clock-stable', '0.5',
                   '--topic', f'/{last_ns}/diff_drive_controller/odom'])

    actions = []
    for idx, (ns, x, y, yaw) in enumerate(ROBOTS):
        robot_description = {
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file,
                         ' use_gazebo:=true', f' robot_ns:={ns}']),
                value_type=str)
        }

        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=ns, output='screen',
            parameters=[robot_description, sim, {'frame_prefix': f'{ns}/'}],
        ))

        # Two gates, as in gazebo.launch.py: wait for the server to advertise
        # /world/<name>/create, then wait again for the GUI to finish building
        # its scene, because a model inserted mid-load is accepted by the server
        # and then silently never drawn.
        spawn = ExecuteProcess(
            cmd=['bash', '-c',
                 f'until gz service -l 2>/dev/null | '
                 f'grep -q "^/world/{WORLD_ENTITY}/create$"; do sleep 2; done; '
                 f'sleep {SPAWN_SETTLE + idx * SPAWN_STAGGER}; '
                 f'exec ros2 run ros_gz_sim create '
                 f'-world {WORLD_ENTITY} '
                 f'-topic /{ns}/robot_description -name {ns} '
                 f'-x {x} -y {y} -z {SPAWN_Z} -Y {yaw}'],
            output='screen',
        )
        actions.append(spawn)

        # --controller-manager-timeout is raised well above the spawner's default
        # 10 s because every robot's chain starts at once and all of their
        # controller_managers live inside the ONE gz sim process. At four robots
        # that contention already lost a controller outright: a spawner gave up
        # mid-load with "Failed getting a result from calling
        # /r4/controller_manager/load_controller in 10.0" and died, leaving that
        # controller unconfigured while the other robots came up complete.
        # Nothing was wrong with that robot -- it just queued last. The timeout
        # costs nothing when the call returns promptly.
        def spawner(name, extra=None, _ns=ns):
            args = [name, '--controller-manager', f'/{_ns}/controller_manager',
                    '--controller-manager-timeout', '120']
            if extra:
                args += ['--param-file', extra]
            return Node(package='controller_manager', executable='spawner',
                        arguments=args, output='screen')

        jsb = spawner('joint_state_broadcaster')
        arm = spawner('arm_controller')
        grip = spawner('gripper_controller')
        diff = spawner('diff_drive_controller', diff_drive_frames(ns))
        actions += [
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
            RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
            RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[grip])),
            RegisterEventHandler(OnProcessExit(target_action=grip, on_exit=[diff])),
        ]

        nav_actions = []
        if ns in NAV_ROBOTS:
            # EKF: the single source of <ns>/odom -> <ns>/base_link, because the
            # diff drive controller's own odom TF is disabled.
            #
            # ekf.yaml MUST go through the same rewriter as the Nav2 files, not
            # just get its frames overridden inline. Its root key is
            # `ekf_filter_node:`, which does not match a node at
            # /r1/ekf_filter_node, so the file is silently ignored and the filter
            # starts on defaults -- where every odom0_config/imu0_config entry is
            # false. It then publishes nothing, odom -> base_link never appears,
            # and the only visible symptom is AMCL dropping every scan with
            # "queue is full".
            nav_actions.append(Node(
                package='robot_localization', executable='ekf_node',
                name='ekf_filter_node', namespace=ns, output='screen',
                parameters=[namespaced_params(
                    os.path.join(desc_share, 'config', 'ekf.yaml'), ns), sim],
            ))

            # AMCL supplies map -> <ns>/odom. The initial pose is the spawn pose:
            # the map was generated from the world's own geometry in world
            # coordinates, so the map frame IS the Gazebo world frame and no
            # reseeding is needed.
            amcl_params = namespaced_params(
                amcl_yaml, ns,
                overrides={'amcl': {'initial_pose.x': x,
                                    'initial_pose.y': y,
                                    'initial_pose.yaw': yaw}})
            nav_actions.append(Node(
                package='nav2_amcl', executable='amcl', name='amcl',
                namespace=ns, output='screen',
                parameters=[amcl_params, sim],
                remappings=[('map', '/map')]))

            nav_params = namespaced_params(nav2_yaml, ns)
            for pkg, exe in (('nav2_controller', 'controller_server'),
                             ('nav2_smoother', 'smoother_server'),
                             ('nav2_planner', 'planner_server'),
                             ('nav2_behaviors', 'behavior_server'),
                             ('nav2_bt_navigator', 'bt_navigator')):
                remaps = [('map', '/map')]
                if exe == 'controller_server':
                    remaps.append(
                        ('cmd_vel',
                         f'/{ns}/diff_drive_controller/cmd_vel_unstamped'))
                nav_actions.append(Node(
                    package=pkg, executable=exe, name=exe, namespace=ns,
                    output='screen', parameters=[nav_params, sim],
                    remappings=remaps))

            # THE LIFECYCLE MANAGER STARTS AFTER THE NODES IT MANAGES, AND THE
            # STACKS ARE STAGGERED AGAINST EACH OTHER.
            #
            # nav2_lifecycle_manager configures its nodes as fast as it can and
            # does not retry: if a node is still constructing, or the machine is
            # too busy to answer a change_state call, the manager stops where it
            # is and logs nothing further. Roughly half of all runs died that
            # way, and never in the same place twice -- "Configuring
            # smoother_server", then "Configuring controller_server", then
            # "Configuring planner_server". A different node each time is the
            # signature of contention, not of a broken node.
            #
            # The manager's patience is NOT tunable: Humble's
            # LifecycleServiceClient hardcodes get_state at 2 s and exposes no
            # parameter for it. So the gate waits for the change_state services
            # to appear, AND for the previous robot's stack to be SERVING --
            # navigate_to_pose exists only once that bt_navigator is active,
            # which is exactly "that stack has finished and the machine is quiet
            # again". A purely time-based stagger only chooses which robot loses.
            nav_order = list(NAV_ROBOTS)
            prev_ns = (nav_order[nav_order.index(ns) - 1]
                       if nav_order.index(ns) > 0 else None)
            gate_args = ['--label', f'{ns} nav services', '--timeout', '300'] \
                + [arg for n in ['amcl'] + NAV_NODES
                   for arg in ('--service', f'/{ns}/{n}/change_state')]
            if prev_ns is not None:
                gate_args += ['--action', f'/{prev_ns}/navigate_to_pose']
            lifecycle_gate = Node(
                package='pickplace_arm_bringup', executable='wait_for',
                name='wait_for_nav_services', namespace=ns, output='screen',
                parameters=[sim], arguments=gate_args)

            # AUTOSTART IS OFF AND nav_bringup DRIVES THE LIFECYCLE INSTEAD.
            # Neither gate above can fix the real defect: with autostart the
            # manager makes ONE pass, each change_state call bounded by that
            # hardcoded 2 s, and one slow answer stops it for good having logged
            # nothing -- while the abandoned node reaches `inactive` by itself
            # moments later. nav_bringup retries, which is the one thing the
            # manager will not do. See its module docstring.
            managed = ['amcl'] + NAV_NODES
            # Registered BEFORE the gate is appended, so the handler is in place
            # before the process it watches can possibly exit.
            nav_actions.append(RegisterEventHandler(
                OnProcessExit(target_action=lifecycle_gate, on_exit=[
                    Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_navigation', namespace=ns,
                        output='screen',
                        parameters=[sim, {'autostart': False,
                                          'bond_timeout': 0.0,
                                          'node_names': managed}]),
                    Node(
                        package='pickplace_arm_bringup', executable='nav_bringup',
                        name='nav_bringup', namespace=ns, output='screen',
                        parameters=[sim],
                        arguments=['--namespace', ns]
                        + [a for n in managed for a in ('--node', n)]),
                ])))
            nav_actions.append(lifecycle_gate)
            # Off the fleet gate rather than this robot's own spawner, so no
            # stack starts until every robot is up. NAV_SETTLE then pushes even
            # the first robot past the last spawn, and NAV_STAGGER keeps the
            # bring-ups from overlapping each other.
            actions.append(RegisterEventHandler(
                OnProcessExit(target_action=fleet_gate, on_exit=[TimerAction(
                    period=float(NAV_SETTLE + idx * NAV_STAGGER),
                    actions=nav_actions)])))

        # MoveIt, one move_group per robot, so every one of them can use its arm.
        #
        # THE URDF HANDED TO MoveIt IS NOT LINK-PREFIXED, AND THAT IS ON PURPOSE.
        # move_group runs inside the robot's namespace and nothing outside it
        # ever sees its link names, so `base_link` here and `r1/base_link` in the
        # TF tree are two consistent namespaces that never meet. Prefixing the
        # links instead would drag in the SRDF, the ros2_control joint list, the
        # DetachableJoint's parent_link and the Clearpath xacro, which has no
        # prefix argument at all.
        #
        # THIS IS THE HEAVIEST NODE PER ROBOT, and three of them is more than the
        # abandoned four-robot attempt ever ran (it could afford two). If the
        # real-time factor is the problem, this is the first thing to cut back.
        if ns in ARM_ROBOTS:
            moveit_config = (
                MoveItConfigsBuilder('pickplace_arm',
                                     package_name='pickplace_arm_moveit_config')
                .robot_description(
                    mappings={'use_gazebo': 'true', 'robot_ns': ns})
                .to_moveit_configs())
            # HELD BACK TO THE FLEET GATE, LIKE THE NAVIGATION STACKS. Ungated,
            # this starts at t=0 and spends its whole construction competing with
            # Gazebo streaming 175 models in -- the heaviest node in the system,
            # against the one thing that has a hard 2 s deadline downstream.
            #
            # It starts on the gate with NO extra delay while the Nav2 stacks
            # wait NAV_SETTLE behind it, so move_group gets the quiet window
            # between the last spawn and the first lifecycle manager instead of
            # overlapping either. Nothing needs an arm until a mission runs.
            actions.append(RegisterEventHandler(
                OnProcessExit(target_action=fleet_gate, on_exit=[Node(
                    package='moveit_ros_move_group', executable='move_group',
                    namespace=ns, output='screen',
                    parameters=[moveit_config.to_dict(), sim,
                                {'trajectory_execution.allowed_start_tolerance': 0.1}])])))

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        prefix=('env -u GTK_PATH -u GTK_EXE_PREFIX -u LOCPATH '
                '-u GDK_PIXBUF_MODULE_FILE -u GDK_PIXBUF_MODULEDIR '
                '-u GIO_MODULE_DIR -u GTK_IM_MODULE_FILE'),
        # GENERATED FROM THE FLEET LIST, not a committed .rviz file. The
        # single-robot configs describe one robot on unprefixed topics and
        # cannot be pointed at three at once; written out by hand for a fleet
        # the file would be thousands of lines of near-duplicate YAML, and the
        # next robot added to fleet_layout.ROBOTS would silently not appear in
        # it. See fleet_rviz.py, including which displays ship disabled and why.
        arguments=['-d', fleet_rviz_config(
            ROBOTS, arm_robots=ARM_ROBOTS, nav_robots=NAV_ROBOTS,
            view_centre=FORMATION_CENTRE)],
        parameters=[sim])

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        *[AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', p) for p in
          (os.path.dirname(desc_share),
           os.path.join(desc_share, 'models'),
           os.path.join(desc_share, 'aws_hospital_models'))],
        SetEnvironmentVariable('GZ_FUEL_CACHE_PATH',
                               os.path.join(desc_share, 'fuel_cache')),
        # The same DDS workaround the single-robot launches carry, and a fleet
        # needs it more than they do: three robots is roughly thirty nodes, and
        # default FastDDS discovery starts losing endpoints at that scale. The
        # symptom is not a crash -- it is one robot silently never receiving a
        # goal its action client believed it sent, while its identical siblings
        # drive off normally.
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.join(bringup_share, 'config', 'fastdds_udp_only.xml')),
        gazebo,
        bridge,
        map_server,
        map_lifecycle,
        rviz,
        fleet_gate,
        *actions,
    ])
