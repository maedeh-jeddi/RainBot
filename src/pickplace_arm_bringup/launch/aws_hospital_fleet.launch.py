import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, SetEnvironmentVariable, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

from pickplace_arm_bringup.ns_params import diff_drive_frames, namespaced_params

# THE FLEET. One tuple per robot - (namespace, x, y, yaw) - and everything else
# in this file is generated from it, so adding a fifth is one line.
#
# A ring of radius 2.25 m around (0, 11.50), out in the open lobby, every robot
# yawed to look at that centre and therefore at the robot opposite it. The lobby
# planter used to stand at (0, 7.75) and was removed to clear the middle; if it
# ever goes back into aws_hospital.sdf, this ring is far enough north not to
# care.
#
# THE CENTRE IS NOT THE MIDDLE OF THE LOBBY, and that is the point. A ring on
# (0, 7.75) put its south robot at (0, 5.00), which is 0.26 m from the nurses'
# station - inside the desk, parked behind reception. Nothing caught it because
# the clearance metric measured distance to each prop's ORIGIN, and that desk's
# origin is at (0, 1.5) while its collision mesh reaches y = 6.30. Distance to a
# model's origin is not distance to the model. This ring is placed against the
# desk's actual footprint, and against the walls via the map Nav2 plans on.
#
# Measured, in metres:
#
#            map    desk    props   actor paths
#   west     3.47   5.49     4.90      0.50
#   east     3.46   5.49     4.30      0.50
#   north    3.12   7.45     3.13      2.75
#   south    6.35   2.95     4.86      1.75
#
# The 0.50 m to an actor path on the east-west pair does not matter physically -
# Gazebo <actor>s carry no collision, so a walker passes THROUGH a parked robot
# rather than into it - but it does put a moving obstacle in those two robots'
# costmaps, which is worth knowing when they plan their first path out.
RING_CENTRE = (0.0, 11.50)
ROBOTS = [
    ('r1', -2.25, 11.50, 0.0),
    ('r2', 2.25, 11.50, 3.14159265),
    ('r3', 0.0, 13.75, -1.5707963),
    ('r4', 0.0, 9.25, 1.5707963),
]

# --- bringing it up faster while developing ----------------------------------
#
# EVERY DELAY BELOW IS THERE BECAUSE SOMETHING FAILED WITHOUT IT, so none of
# them change by default: with no environment set, this file behaves exactly as
# it did. What the overrides buy is iteration speed, and the measured budget
# says where the time actually goes - launch to the mission's first Nav2 goal is
# 150 s, identical across runs, spent as
#
#   ~45 s  SPAWN_SETTLE before the first robot is inserted
#    36 s  SPAWN_STAGGER, 12 s for each robot after the first
#   ~20 s  the controller chains finishing
#    45 s  NAV_SETTLE before the first navigation stack
#   ~24 s  lifecycle bring-up and the mission's own gates
#
# FLEET_ROBOTS is the biggest single lever and the safest. r3 and r4 never
# navigate and never manipulate - they are furniture that happens to be a robot
# - so a run that only exercises r1 and r2 does not need them inserted at all,
# and dropping them removes 24 s of stagger plus their share of the contention
# the stagger exists to spread. They stay in the MAP either way (see
# parked_robots in hospital_aws_layout), so the planner still routes around
# where they would have been, which is the conservative direction to be wrong in.
#
# FLEET_NAV_SETTLE is the next one. That 45 s is a fixed guess sitting on top of
# a condition that is already implemented and already waited on - the
# change_state gate below waits for each robot's services to exist AND for the
# previous robot's navigate_to_pose to be serving. With two robots the gate does
# the whole job, so the constant is mostly redundant; with four it is still
# earning its keep, which is why it is not simply deleted.
#
#   FLEET_ROBOTS=r1,r2 FLEET_NAV_SETTLE=0 FLEET_SPAWN_SETTLE=30 \
#     ros2 launch pickplace_arm_bringup mission_hospital_relay.launch.py
#
# brings the first Nav2 goal in at roughly 90 s instead of 150.
_want = os.environ.get('FLEET_ROBOTS')
if _want:
    _keep = [n.strip() for n in _want.split(',') if n.strip()]
    _known = {n for n, *_ in ROBOTS}
    _unknown = [n for n in _keep if n not in _known]
    if _unknown:
        raise RuntimeError(
            f'FLEET_ROBOTS names {_unknown} which are not in ROBOTS {sorted(_known)}')
    ROBOTS = [r for r in ROBOTS if r[0] in _keep]

# WHICH ROBOTS GET A move_group. Every robot carries an FR3, but move_group is
# by far the heaviest node per robot and this machine cannot run four of them.
# Measured: with all four, the one-minute load average reached 71.8 and the
# controller spawners stopped being able to hold their controllers up at all -
# r2 and r4 went from one and two active controllers back to zero while the
# machine thrashed. Nothing crashed and no spawner timed out; the nodes were
# simply starved.
#
# Two is what the relay needs - one robot collects, the other delivers - so the
# east-west pair, which already face each other, carry the arms. The other two
# still drive, sense and navigate; they just cannot pick anything up.
ARM_ROBOTS = ('r1', 'r2')

# WHICH ROBOTS GET A NAVIGATION STACK. Same reason, bigger numbers: Nav2 plus
# AMCL plus the EKF costs about 230% of one CPU per robot AND IT COSTS THAT
# WHILE THE ROBOT IS STANDING STILL - the costmaps, the behaviour tree and the
# particle filter all run on their own timers whether or not there is a goal.
# Measured on a 16-core machine with the GUI up: four navigating robots plus two
# move_groups came to roughly 1200% of the 1600% available, and controllers
# stopped staying up.
#
# So the two arm robots navigate and the other two are physically real but
# parked - they spawn, they render, they hold their pose, their controllers
# work. Add a name here to give it a stack back; the ring layout does not
# change either way.
NAV_ROBOTS = ('r1', 'r2')

SPAWN_Z = 0.14
WORLD_ENTITY = 'aws_hospital'

# HOW LONG TO WAIT BEFORE INSERTING THE FIRST ROBOT, AND HOW FAR APART TO SPACE
# THE REST.
#
# The settle is the GUI gate described at the spawn below: a model inserted
# while the viewport is still streaming 175 models never gets drawn.
#
# The stagger exists for a different reason and is not cosmetic. Every robot's
# controller chain starts when its spawn returns, and all four
# controller_managers live inside the ONE gz sim process. Spawned together, the
# four chains hit that process at the same instant and the losers do not just
# wait - they die: "Could not contact service /r1/controller_manager/
# list_controllers", spawner exits 1, and that robot has no drive controller for
# the rest of the run. Raising the spawner timeout alone did not fix it, because
# the problem is contention rather than a slow answer. Offsetting each robot by
# a few seconds spreads both the Gazebo insertion and the controller bring-up.
SPAWN_SETTLE = int(os.environ.get('FLEET_SPAWN_SETTLE', 45))
SPAWN_STAGGER = int(os.environ.get('FLEET_SPAWN_STAGGER', 12))
# Seconds between one robot's Nav2 stack and the next. How long a robot's own
# servers need before its lifecycle manager may touch them is deliberately NOT a
# constant any more - see the change_state gate below, which waits on the
# services themselves rather than guessing at a duration.
NAV_STAGGER = int(os.environ.get('FLEET_NAV_STAGGER', 25))
# EVERY ROBOT WAITS, INCLUDING THE FIRST. The stagger alone was idx * NAV_STAGGER,
# which gives robot zero no delay at all - and robot zero was the one that kept
# failing while its staggered siblings came up clean. Observed directly: r2
# reached "Managed nodes are active" while r1 sat on "Configuring
# controller_server", with the machine only lightly loaded, so this is ordering
# and not starvation. r1's controllers finish first, so its Nav2 bring-up starts
# while r3 and r4 are still being inserted into Gazebo and claiming controllers
# of their own. NAV_SETTLE pushes the first stack past the last spawn.
NAV_SETTLE = int(os.environ.get('FLEET_NAV_SETTLE', 45))
NAV_NODES = ['controller_server', 'smoother_server', 'planner_server',
             'behavior_server', 'bt_navigator']


def generate_launch_description():
    """A fleet of independently navigating robots in the AWS hospital.

    STAGES 1 AND 2 of multi-robot support, and the base every later stage runs
    on. Each robot gets its own sensor topics, TF tree, controller_manager, EKF,
    AMCL and full Nav2 stack; they share one map_server and one map frame.

    The stages above this one are missions, not infrastructure, and they live in
    their own launch files: mission_hospital_aws.launch.py runs the whole
    sample-transport route with one robot (stages 3-4), and
    mission_hospital_relay.launch.py splits that same route across two with a
    handover in the middle (stage 5). Both include THIS file unchanged.

    COST PER ROBOT. A full Nav2 stack is five nodes, plus AMCL, an EKF, a
    robot_state_publisher and four controller spawners - so the fleet size in
    ROBOTS multiplies about ten processes. The simulator side scales worse: each
    robot carries two RGB-D cameras, and RGB-D rendering is already the most
    expensive thing in this world (see the rate comment in
    pickplace_arm.gazebo.xacro). Watch the real-time factor when you grow the
    fleet; if it falls far enough the LIDAR starves and AMCL follows it down.

    HOW THE TWO ARE KEPT APART:

      description   robot_ns in pickplace_arm.urdf.xacro prefixes the sensor
                    topics, the sensor frames and the grasp topics, and
                    namespaces the gz_ros2_control plugin
      TF            robot_state_publisher's frame_prefix, matched to those
                    sensor frames; each robot owns an r<N>/odom island
      navigation    ns_params.py rewrites nav2_params.yaml and amcl_tugbot.yaml
                    per robot BY VALUE, so `odom` becomes r1/odom while `map`
                    stays shared - see that module for why a key-based rewrite
                    cannot do this

    WHAT MAKES THEM AVOID EACH OTHER: nothing explicit. Each robot's LIDAR sees
    the other as an obstacle and its local costmap plans around it. That is
    enough for two robots in a building this size; it is NOT enough to stop two
    robots meeting head-on in a corridor too narrow to pass, which is a
    coordination problem and belongs to a later stage.
    """
    desc_share = get_package_share_directory('pickplace_arm_description')
    bringup_share = get_package_share_directory('pickplace_arm_bringup')
    xacro_file = os.path.join(desc_share, 'urdf', 'pickplace_arm.urdf.xacro')
    world_file = os.path.join(desc_share, 'worlds', 'aws_hospital.sdf')
    map_yaml = os.path.join(bringup_share, 'maps', 'aws_hospital.yaml')
    nav2_yaml = os.path.join(bringup_share, 'config', 'nav2_params.yaml')
    amcl_yaml = os.path.join(bringup_share, 'config', 'amcl_tugbot.yaml')
    sim = {'use_sim_time': True}

    gz_flags = '-s -r ' if os.environ.get('HEADLESS') == '1' else '-r '
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_flags + world_file,
                          'gz_version': '8'}.items(),
    )

    # One map_server for both robots, in the global namespace. The map frame is
    # the only thing the two share, and it is what makes their separate odom
    # islands comparable - r1 knowing where r2 is starts here.
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[sim, {'yaml_filename': map_yaml}])
    map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[sim, {'autostart': True, 'node_names': ['map_server']}])

    # ONLY BRIDGE THE CAMERAS OF ROBOTS THAT ACTUALLY PICK THINGS UP.
    #
    # Each robot carries two RGBD cameras publishing 640x480 organised point
    # clouds at 15 Hz. One such cloud is about 5 MB, so ONE camera is roughly
    # 75 MB/s, and bridging all four robots' pairs put something like 600 MB/s
    # through parameter_bridge and DDS. Nothing carries that: the clouds arrive
    # in bursts with long gaps, and the effect lands on the one thing that needs
    # them continuously, the visual servo -
    #
    #   [detect] no point cloud received before timeout      x7 of 12 tries
    #   [claw] lost the box -- aborting approach
    #
    # It also starves everything sharing the transport, which is why the drive
    # was sluggish. `ros2 topic hz` could not collect 8 messages in 30 s.
    #
    # r3 and r4 never navigate and never manipulate (see NAV_ROBOTS/ARM_ROBOTS)
    # - they are parked furniture that happens to be a robot - so their cameras
    # are pure cost. Dropping them halves the load at zero functional cost.
    # Their scan and imu stay bridged: cheap, and useful for inspection.
    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    for ns, _, _, _ in ROBOTS:
        bridge_args += [
            f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ]
        if ns in ARM_ROBOTS:
            bridge_args += [
                f'/{ns}/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/{ns}/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/{ns}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                f'/{ns}/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                f'/{ns}/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/{ns}/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                f'/{ns}/front_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ]
        bridge_args += [
            # Grasp control, ROS -> gz (']'), prefixed so one robot's gripper
            # cannot weld a prop to the other's.
            f'/{ns}/sample_rack/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            f'/{ns}/sample_rack/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            f'/{ns}/box_red/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            f'/{ns}/box_red/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            f'/{ns}/box_green/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            f'/{ns}/box_green/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            f'/{ns}/box_blue/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            f'/{ns}/box_blue/detach@std_msgs/msg/Empty]gz.msgs.Empty',
        ]
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  arguments=bridge_args, output='screen')

    # GATE THE NAVIGATION STACKS ON A CONDITION, NOT ON A CLOCK.
    #
    # These used to start a fixed number of seconds after each robot's
    # controllers came up, and the first robot in ROBOTS failed almost every
    # time while its staggered siblings came up clean - r2 reaching "Managed
    # nodes are active" while r1 sat on "Configuring amcl", or
    # controller_server, or planner_server, a different node each run. The
    # first stack is simply the one that lands while Gazebo is still inserting
    # the other robots and handing out their controllers.
    #
    # Tuning the delay is a losing game: the world gate measured 63 s early in a
    # session and 104 s hours later on the same machine, so any constant that
    # works now is wrong later. The condition that actually matters is that the
    # LAST robot's LAST controller is publishing - at which point every robot has
    # spawned and every controller_manager has finished handing out controllers.
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

        # --controller-manager-timeout is raised well above the spawner's
        # default 10 s because every robot's chain starts at once and all of
        # their controller_managers live inside the ONE gz sim process. At four
        # robots that contention already lost a controller: r4's
        # gripper_controller spawner gave up mid-load with "Failed getting a
        # result from calling /r4/controller_manager/load_controller in 10.0"
        # and died, leaving that controller unconfigured while the other three
        # robots came up complete. Nothing was wrong with r4 - it just queued
        # last. The timeout costs nothing when the call returns promptly.
        def spawner(name, extra=None):
            args = [name, '--controller-manager', f'/{ns}/controller_manager',
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

        # EKF: the single source of <ns>/odom -> <ns>/base_link, because the
        # diff drive controller's own odom TF is disabled.
        #
        # ekf.yaml MUST go through the same rewriter as the Nav2 files, not just
        # get its frames overridden inline. Its root key is `ekf_filter_node:`,
        # which does not match a node at /r1/ekf_filter_node, so the file is
        # silently ignored and the filter starts on defaults - where every
        # odom0_config/imu0_config entry is false. It then publishes nothing,
        # odom -> base_link never appears, and the only visible symptom is AMCL
        # dropping every scan with "queue is full". The rewriter also maps
        # `odom` -> r1/odom and the input topics, so no manual overrides are
        # needed; `map` is left alone, which is correct here too.
        # THE WHOLE NAVIGATION STACK IS HELD BACK UNTIL THE ROBOT IS DRIVEABLE.
        # Started at launch time it races the world load, and the failure is
        # ugly: nav2_lifecycle_manager's service client times out waiting for
        # amcl/get_state while Gazebo is still streaming 175 models in, and it
        # does not retry - it logs "Failed to bring up all requested nodes.
        # Aborting bringup." and stops, leaving a robot with no
        # navigate_to_pose server at all. Hanging these off the last controller
        # spawner costs nothing and removes the race: by then the robot exists,
        # its controllers are active and the world has finished loading.
        nav_actions = []
        if ns in NAV_ROBOTS:
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
                        ('cmd_vel', f'/{ns}/diff_drive_controller/cmd_vel_unstamped'))
                nav_actions.append(Node(
                    package=pkg, executable=exe, name=exe, namespace=ns,
                    output='screen', parameters=[nav_params, sim],
                    remappings=remaps))

            # bond_timeout 0.0, as in mission_hospital.launch.py, and for a
            # sharper reason here: two robots configure their Nav2 stacks at the
            # same instant, while Gazebo is still streaming 175 models in. Under
            # that load r2's smoother_server took too long to answer its own
            # change_state call, the manager gave up waiting on it, and the
            # bringup stopped dead at "Configuring smoother_server" - no error,
            # no crash, just a robot that never gets a navigate_to_pose server
            # while its identical twin comes up fine.
            # THE LIFECYCLE MANAGER STARTS AFTER THE NODES IT MANAGES, AND THE
            # TWO ROBOTS' STACKS ARE STAGGERED AGAINST EACH OTHER.
            #
            # nav2_lifecycle_manager configures its nodes as fast as it can and
            # does not retry: if a node is still constructing, or the machine is
            # too busy to answer a change_state call, the manager stops where it
            # is and logs nothing further. Roughly half of all runs died that
            # way, and never in the same place twice - "Configuring
            # smoother_server", then "Configuring controller_server", then
            # "Configuring planner_server". Different node each time is the
            # signature of contention, not of a broken node.
            #
            # A FIXED SETTLE WAS THE WRONG SHAPE. LIFECYCLE_SETTLE was 15 s, on
            # the assumption that the five servers would be constructed by then.
            # Measured 2026-08-17 they were not: the manager logged "Configuring
            # controller_server" and stopped there for good, while that server
            # went on to reach `inactive` by itself moments later. The node was
            # healthy - the manager had already abandoned it.
            #
            # The manager's patience is NOT tunable. Humble's
            # LifecycleServiceClient hardcodes get_state at 2 s and exposes no
            # parameter for it, so the only lever left is to not call a node
            # that is still constructing. Wait for the change_state services
            # themselves to appear - the condition that actually matters - in
            # place of a constant that is wrong on any other machine.
            #
            # AND WAIT FOR THE PREVIOUS ROBOT'S STACK TO BE SERVING. NAV_STAGGER
            # alone was not enough, measured 2026-08-17: with r1 fixed and going
            # first, r2 started its 25 s later into a machine now running r1's
            # whole Nav2 stack AND both move_groups - load 12.84 - and died at
            # "Configuring behavior_server". The same failure, one node further
            # along. The contention did not go away, it moved to whoever goes
            # second, which is what any purely time-based stagger will keep
            # doing. So chain on the condition: navigate_to_pose exists only
            # once the previous robot's bt_navigator is active, which is exactly
            # "that stack has finished and the machine is quiet again".
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
            #
            # Neither gate above can fix the real defect: with autostart the
            # manager makes ONE pass, each change_state call bounded by a 2 s
            # deadline Humble hardcodes and exposes no parameter for (confirmed
            # against a live manager - the only params are autostart,
            # bond_timeout, bond_respawn_max_duration,
            # attempt_respawn_reconnection, node_names). One slow answer and it
            # stops for good, having logged nothing, and every time it did so
            # the abandoned node reached `inactive` by itself moments later.
            #
            # Three runs on 2026-08-17 stalled at controller_server, then
            # behavior_server, then amcl - and on a different robot each time.
            # Delays only choose the loser. nav_bringup retries, which is the
            # one thing the manager will not do. See its module docstring.
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
            # the first robot past the last spawn, and NAV_STAGGER keeps the two
            # bring-ups from overlapping each other.
            #
            # NAV_SETTLE HAD BEEN DEFINED, DOCUMENTED, AND NEVER WIRED IN. The
            # period was idx * NAV_STAGGER, which gives r1 - idx 0 - no delay at
            # all, and r1 is exactly the robot that kept failing. The fix had
            # been worked out already; it just never reached the TimerAction.
            actions.append(RegisterEventHandler(
                OnProcessExit(target_action=fleet_gate, on_exit=[TimerAction(
                    period=float(NAV_SETTLE + idx * NAV_STAGGER),
                    actions=nav_actions)])))

        # MoveIt, one move_group per robot, so any of them can use its arm.
        #
        # THE URDF HANDED TO MoveIt IS NOT LINK-PREFIXED, AND THAT IS ON
        # PURPOSE. move_group runs inside the robot's namespace and nothing
        # outside it ever sees its link names, so `base_link` here and
        # `r1/base_link` in the TF tree are two consistent namespaces that never
        # meet. Prefixing the links instead would drag in the SRDF, the
        # ros2_control joint list, the DetachableJoint's parent_link and the
        # Clearpath xacro, which has no prefix argument at all. See tf_frame()
        # in pick_and_place.py for the rule the mission code follows.
        #
        # This is the heaviest node per robot. If the real-time factor becomes
        # the problem, this is the first thing to launch for only the robots
        # that actually need an arm.
        if ns in ARM_ROBOTS:
            moveit_config = (
                MoveItConfigsBuilder('pickplace_arm',
                                     package_name='pickplace_arm_moveit_config')
                .robot_description(
                    mappings={'use_gazebo': 'true', 'robot_ns': ns})
                .to_moveit_configs())
            # HELD BACK TO THE FLEET GATE, LIKE THE NAVIGATION STACK.
            #
            # Ungated, this started at t=0 and spent its whole construction
            # competing with Gazebo streaming 175 models in - two of them, of
            # the heaviest node in the system, against the one thing that has a
            # hard 2 s deadline downstream. Load reached 23.95 on 16 cores.
            #
            # It starts on the gate with NO extra delay while the Nav2 stacks
            # wait NAV_SETTLE behind it, so move_group gets the quiet window
            # between the last spawn and the first lifecycle manager instead of
            # overlapping either. Nothing needs the arm until the mission runs,
            # which is much later still.
            actions.append(RegisterEventHandler(
                OnProcessExit(target_action=fleet_gate, on_exit=[Node(
                    package='moveit_ros_move_group', executable='move_group',
                    namespace=ns, output='screen',
                    parameters=[moveit_config.to_dict(), sim,
                                {'trajectory_execution.allowed_start_tolerance': 0.1}])])))

    return LaunchDescription([
        *[AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', p) for p in
          (os.path.dirname(desc_share),
           os.path.join(desc_share, 'models'),
           os.path.join(desc_share, 'aws_hospital_models'))],
        SetEnvironmentVariable('GZ_FUEL_CACHE_PATH',
                               os.path.join(desc_share, 'fuel_cache')),
        # The same DDS workaround the mission launches carry, and a fleet needs
        # it more than they do: four robots is roughly forty nodes, and on this
        # machine default FastDDS discovery starts losing endpoints at that
        # scale. The symptom is not a crash - it is one robot silently never
        # receiving a goal its action client believed it sent, while its three
        # identical siblings drive off normally.
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.join(bringup_share, 'config', 'fastdds_udp_only.xml')),
        gazebo,
        bridge,
        map_server,
        map_lifecycle,
        fleet_gate,
        *actions,
    ])
