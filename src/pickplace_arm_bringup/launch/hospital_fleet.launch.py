import os
import shlex
import subprocess
import tempfile

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
#
# FLEET_ROBOTS itself is applied in fleet_layout, not here, so the bridge, the
# rack detaches, the mission nodes and the task manager all see the same fleet.

# HOW LONG TO WAIT BEFORE INSERTING THE FIRST ROBOT, AND HOW FAR APART TO SPACE
# THE REST.
#
# THE SETTLE IS A GUI CONCERN ONLY, so headless pays nothing for it. Gazebo
# exposes no "GUI scene loaded" signal, and a model inserted while the viewport
# is still streaming 175 models is accepted by the server and then never drawn --
# so with a GUI this stays a measured delay. With `-s` there is no viewport to
# lose the robot in, and the spawn is already gated on the server advertising
# /world/<name>/create, which is the condition that actually matters.
#
# 20 rather than the 45 it used to be. 45 was inherited from a period when this
# machine was being crippled by an orphaned RViz (see the note on load in
# fleet_layout) and everything was measured against a thrashing box.
SPAWN_SETTLE = int(os.environ.get(
    'FLEET_SPAWN_SETTLE', 0 if os.environ.get('HEADLESS') == '1' else 20))
# The stagger is NOT cosmetic and stays: every robot's controller chain starts
# when its spawn returns, and all of their controller_managers live inside the
# ONE gz sim process. Spawned together the losers do not wait, they die --
# "Could not contact service /r1/controller_manager/list_controllers", and that
# robot has no drive controller for the rest of the run. 6 s is enough to spread
# both the insertion and the controller bring-up.
SPAWN_STAGGER = int(os.environ.get('FLEET_SPAWN_STAGGER', 6))

# NAV_STAGGER IS 12, AND ZERO WAS THE SINGLE BIGGEST CAUSE OF THIS FLEET
# FAILING TO COME UP.
#
# The previous value was 0, on the reasoning that ACTIVATION is already
# serialised by the gate below, so the NODES might as well construct in
# parallel. The serialisation part is true and the conclusion is wrong: what
# matters is not when the stacks activate, it is that starting them together
# creates eighteen DDS participants at the same instant, on top of the forty
# this system already runs. Discovery is pairwise, and the storm loses exactly
# the small messages bring-up depends on.
#
# The symptom was never a crash. Nodes came up healthy and simply could not be
# reached: /r2/bt_navigator and /r2/planner_server were ABSENT from
# `ros2 node list` while their own services were listed, their processes idling
# at 2% CPU, waiting for a configure request that never arrived. The lifecycle
# manager logged "Configuring bt_navigator" and nothing after it, ever. That is
# the same TRANSIENT_LOCAL-over-shared-memory unreliability that already cost
# this project the map (map_pump.py) and the robot descriptions (the -file spawn
# below), showing up in service calls instead of latched topics.
#
# Measured back to back, same machine, same world, GUI and rviz both running:
#
#             NAV_STAGGER=0                     NAV_STAGGER=12
#     r1      active t+64.5s                    active t+56.3s
#     r2      attempt FAILED after 100s         active t+70.9s
#     r3      (never reached an attempt)        active t+83.3s
#     mission never dispatched                  DISPATCHED t+92.3s
#
# Every robot came up on its FIRST attempt, each taking about three seconds,
# where before a single failed attempt cost 100 s and four of them cost six and
# a half minutes -- usually ending with two robots up and the mission never
# starting, which is exactly what this looked like from the outside.
#
# So the 24 s this spends is not a tax on a working system, it is the
# difference between a working one and a coin flip.
# WHICH DDS PROFILE THE WHOLE FLEET USES.
#
# Back to UDP-only. fastdds_shm.xml was adopted for throughput, and its
# measurements were real -- but it re-introduced the exact failure
# fastdds_udp_only.xml was written to remove, and that file's own docstring
# predicts what was measured here line for line: a participant hits
#
#     RTPS_TRANSPORT_SHM Error: Failed init_port fastrtps_port7445:
#         open_and_lock_file failed
#
# "and then never discovers its topics ... there is no /odom, the EKF
# publishes no odom->base_link". Observed on this fleet, with r3's wheel
# odometry flowing at 49.9 Hz and its IMU at 50.1 Hz, the frame r3/odom did
# not exist AT ALL -- so amcl could not publish map->odom, the global
# costmap blocked in "Timed out waiting for transform from r3/base_link to
# map", planner_server never finished activating, and the robot never came
# up. Clearing /dev/shm first does not prevent it: the port failure happens
# as LATER participants join, not only at startup.
#
# THE THROUGHPUT ARGUMENT HAS MOVED. SHM was chosen when three 480x480
# clouds at 15 Hz were on the bridge; that is now 320x320 at 10 Hz, roughly
# a 70% cut, and rviz no longer starts during bring-up. What SHM was paying
# for is largely gone, and it was being paid for with robots that do not
# start.
DDS_PROFILE = 'fastdds_udp_only.xml'

NAV_STAGGER = int(os.environ.get('FLEET_NAV_STAGGER', 12))
NAV_SETTLE = int(os.environ.get('FLEET_NAV_SETTLE', 0))

# Seconds between one robot's move_group and the next. See the event handler at
# the bottom of this file for what this is spreading and why it costs nothing.
# 15 s is enough for a move_group to get through loading its planning pipelines
# before the next starts; it is not tuned finer than that because the arms have
# minutes of slack before anything needs them.
ARM_STAGGER = int(os.environ.get('FLEET_ARM_STAGGER', 15))
NAV_NODES = ['controller_server', 'smoother_server', 'planner_server',
             'behavior_server', 'bt_navigator']


def _rviz_env_prefix(bringup_share):
    """Run rviz2 in a WHITELISTED environment rather than a blacklisted one.

    THE BLACKLIST THIS REPLACES DID NOT WORK. It unset seven GTK/GDK variables
    that a snap-packaged editor exports, and the reasoning was sound as far as
    it went -- but launched from that editor's terminal rviz2 still died every
    single time, so `use_rviz:=true` had never once produced a window:

        rviz2: symbol lookup error:
            /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0:
            undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
        [rviz2] Failed to create an OpenGL context. GLXBadDrawable
        process has died [exit code -11]

    NOT A GRAPHICS PROBLEM, despite what the second line says. glxinfo on the
    same display reports an RTX 2050 with direct rendering and OpenGL 4.6; the
    same rviz2 with the same config runs fine under `env -i`. It is the parent
    environment, and delta-debugging it found the breakage is not one variable
    to unset: GNOME_DESKTOP_SESSION_ID alone is enough to segfault rviz2, and
    removing it from the full environment still leaves other breakers behind.

    That is why this is a whitelist. A blacklist has to enumerate every variable
    any desktop session might ever export, and it silently loses that race --
    which is exactly how this ended up shipping broken. A whitelist enumerates
    what rviz2 NEEDS, which is short, known, and does not grow when someone
    launches from a different terminal.

    FASTRTPS_DEFAULT_PROFILES_FILE is rebuilt here rather than forwarded: this
    launch file sets it with SetEnvironmentVariable further down, so it is not
    in os.environ yet at the moment this prefix string is constructed. Dropping
    it would leave rviz2 the one participant in the fleet not using shared
    memory, subscribing to three robots' point clouds over loopback UDP.
    """
    keep = ('HOME', 'USER', 'DISPLAY', 'XAUTHORITY', 'PATH', 'LD_LIBRARY_PATH',
            'AMENT_PREFIX_PATH', 'PYTHONPATH', 'ROS_DISTRO', 'ROS_VERSION',
            'ROS_PYTHON_VERSION', 'RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID',
            'ROS_LOCALHOST_ONLY', 'ROS_AUTOMATIC_DISCOVERY_RANGE',
            'XDG_RUNTIME_DIR')
    pairs = [f'{k}={os.environ[k]}' for k in keep if os.environ.get(k)]
    if not os.environ.get('XAUTHORITY'):
        pairs.append('XAUTHORITY=' + os.path.expanduser('~/.Xauthority'))
    pairs.append('FASTRTPS_DEFAULT_PROFILES_FILE='
                 + os.path.join(bringup_share, 'config', DDS_PROFILE))
    return 'env -i ' + ' '.join(shlex.quote(pair) for pair in pairs)


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
    # BOND CHECKING IS OFF, AND WITHOUT THIS THE FLEET COMES UP ABOUT HALF THE
    # TIME. nav2_lifecycle_manager holds a bond with each server it manages and
    # kills one that misses a heartbeat for bond_timeout (4 s by default). During
    # bring-up this machine is spawning three robots and starting three Nav2
    # stacks, and map_server -- which has nothing to do but hold one 540x1160
    # grid -- loses that race:
    #
    #   t+218  Have not received a heartbeat from map_server.
    #   t+218  CRITICAL FAILURE: SERVER map_server IS DOWN after not receiving
    #
    # It is then restarted, and THAT is what breaks the robots. /map is latched
    # (TRANSIENT_LOCAL), so a robot whose amcl subscribes while the publisher is
    # alive gets it and never thinks about it again; one that subscribes during
    # a restart window gets nothing and blocks in configure forever. Measured on
    # the run that motivated this: r1 received the map and was navigating 42 s
    # in, while r2 and r3 logged "Creating" and never activated a single node.
    # nav_bringup then spent four attempts each on stacks that could not
    # possibly come up, and the mission gate waited on all three for 465 s.
    #
    # The heartbeat is worth nothing here anyway: this map_server reads one file
    # at startup and serves a latched topic. There is no failure it can have
    # that restarting it fixes, and the restart is itself the failure.
    map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[sim, {'autostart': True, 'node_names': ['map_server'],
                          'bond_timeout': 0.0}])

    # THE SECOND HALF OF GETTING THE MAP TO EVERY ROBOT. Disabling the bond above
    # stopped map_server being KILLED mid-bringup; it did not make the one latched
    # /map sample arrive everywhere. Measured after the bond fix, with no
    # heartbeat failures at all: r2 and r3 received the map, r1 never so much as
    # logged "Subscribed to map topic", and two of three stacks never activated.
    # See map_pump.py -- including why amcl's first_map_only must be true.
    map_pump = Node(
        package='pickplace_arm_bringup', executable='map_pump',
        name='map_pump', output='screen', parameters=[sim],
        arguments=['--period', '2.0', '--duration', '1800'])

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

    # Filled in the loop below, started behind arm_gate at the end.
    arm_actions = []

    # SPAWN FROM A FILE, NOT FROM /<ns>/robot_description.
    #
    # `ros_gz_sim create -topic ...` subscribes to a LATCHED topic and blocks
    # until a sample arrives, and on this stack that sample does not reliably
    # arrive. Measured on a hung bring-up, with r3's robot_state_publisher alive
    # and publishing:
    #
    #     /r3/robot_description   Publisher count: 1   Subscription count: 1
    #     ros2 topic echo /r3/robot_description --once   -> nothing in 20 s
    #     r3's `create` still waiting, 5 minutes in; only 2 of 3 robots spawned
    #
    # That is the same TRANSIENT_LOCAL-over-shared-memory delivery failure that
    # config/fastdds_shm.xml already warns about ("if a participant ever hangs
    # on Waiting messages on topic [/robot_description]"), and clearing
    # /dev/shm beforehand does not prevent it -- this run started from a clean
    # /dev/shm and a load of 1.49.
    #
    # The xacro is expanded once here and written out, so `create` reads a file
    # off disk and cannot be blocked by DDS at all. robot_state_publisher still
    # publishes the topic for everything else that wants it; nothing else NEEDS
    # it before the robot exists.
    urdf_dir = tempfile.mkdtemp(prefix='fleet_urdf_')
    urdf_files = {}
    for ns, *_ in ROBOTS:
        urdf = subprocess.check_output(
            ['xacro', xacro_file, 'use_gazebo:=true', f'robot_ns:={ns}',
             # Nothing in this mission reads the wrist camera; see the note
             # in pickplace_arm.gazebo.xacro.
             'wrist_camera:=false'],
            text=True)
        path = os.path.join(urdf_dir, f'{ns}.urdf')
        with open(path, 'w') as fh:
            fh.write(urdf)
        urdf_files[ns] = path

    actions = []
    for idx, (ns, x, y, yaw) in enumerate(ROBOTS):
        robot_description = {
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file,
                         ' use_gazebo:=true', f' robot_ns:={ns}',
                         ' wrist_camera:=false']),
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
                 f'-file {urdf_files[ns]} -name {ns} '
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
            # HELD BACK UNTIL EVERY NAVIGATION STACK IS SERVING, which is later
            # than it looks and is the point.
            #
            # This used to start on the fleet gate, alongside the Nav2 stacks.
            # Measured on this machine, that is what made three robots
            # impossible: three move_groups came to 61% of a core EACH during
            # the window where nav2_lifecycle_manager is making change_state
            # calls with a 2 s deadline it hardcodes. The second and third
            # robots' stacks then failed every one of nav_bringup's four
            # attempts -- "6 node(s) not active" -- while the one-minute load
            # average climbed past 40 and stayed there. The nodes were healthy;
            # they could not answer in time.
            #
            # NOTHING NEEDS AN ARM UNTIL A ROBOT REACHES A TABLE, which is
            # minutes after it starts driving, so paying for three move_groups
            # during bring-up buys nothing at all. Waiting for the last robot's
            # navigate_to_pose is a CONDITION rather than a delay: a fast
            # machine starts the arms sooner, a slow one still starts them
            # correctly.
            arm_actions.append(Node(
                package='moveit_ros_move_group', executable='move_group',
                namespace=ns, output='screen',
                parameters=[moveit_config.to_dict(), sim,
                            {'trajectory_execution.allowed_start_tolerance': 0.1}]))

    # The condition the arms wait on: every robot's bt_navigator serving, which
    # is exactly "all three stacks are up and the machine is quiet again".
    # WAITS ON nav_ready, NOT ON navigate_to_pose. The action server is
    # advertised at CONFIGURE, so gating on it fired while two of three stacks
    # were still inactive -- and then started three move_groups and rviz into
    # the bring-up they were meant to follow. See nav_bringup.announce_ready().
    arm_gate = Node(
        package='pickplace_arm_bringup', executable='wait_for',
        name='wait_for_arms', output='screen', parameters=[sim],
        arguments=['--label', 'arms', '--timeout', '900']
        + [a for n in NAV_ROBOTS for a in ('--topic', f'/{n}/nav_ready')])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        prefix=_rviz_env_prefix(bringup_share),
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
        # SHARED MEMORY FOR DATA, UDP FOR DISCOVERY -- and for a fleet that is
        # not a micro-optimisation, it is the difference between the machine
        # coping and not. The single-robot launches use fastdds_udp_only.xml,
        # whose own note says the throughput cost of disabling SHM "is not a
        # concern" on one host; with three robots /tf alone runs at 76 Hz into
        # 29 subscribers, so every message was being pushed through loopback UDP
        # twenty-nine times. See config/fastdds_shm.xml for the measurements and
        # for how to clear the stale-segment error that made someone turn SHM
        # off in the first place.
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.join(bringup_share, 'config', DDS_PROFILE)),
        gazebo,
        bridge,
        map_server,
        map_lifecycle,
        map_pump,
        fleet_gate,
        *actions,
        arm_gate,
        # RVIZ STARTS WITH THE ARMS, NOT WITH THE LAUNCH, and this is a
        # reliability fix rather than a cosmetic one.
        #
        # rviz2 is the second most expensive process in this system while it is
        # STARTING: it loads three robots' meshes, builds a display per topic
        # and subscribes to the lot. Measured during bring-up it sat at 166% of
        # a core -- more than the whole ros_gz bridge -- and that lands in the
        # exact window where nav2_lifecycle_manager is making change_state calls
        # against a deadline Humble hardcodes at 2 s. Runs with rviz competing
        # there repeatedly left r2 and r3 with "6 node(s) not active" through
        # four bring-up attempts, while r1 came up first time.
        #
        # Nothing is lost by waiting. arm_gate is already the condition "every
        # robot's navigate_to_pose is serving", so the window rviz misses is one
        # where the robots are not moving and there is nothing to watch. It
        # still opens by itself when the launch file is run, which is what it is
        # there for.
        # THE ARMS ARE STAGGERED AMONG THEMSELVES, not just held behind the
        # gate. Waiting for every nav stack fixed WHEN they start; it did not
        # fix that all three then start in the same instant, and by the comment
        # above each one costs 61% of a core while it constructs its planning
        # pipelines. Three of those together is where this bring-up's thermal
        # peak lives.
        #
        # Measured, headless, no rviz: the fleet held 67-76 C and load 1.2 for
        # a full minute with the world and all three robots running, then went
        # 83 C / load 3.4 -> 93 C / load 7.8 in the twenty seconds where the
        # nav stacks finished and the move_groups came up together. The
        # simulator was never the problem; the coincidence was.
        #
        # ARM_STAGGER spreads that same work instead of removing it, so nothing
        # is given up: an arm is still idle until a robot reaches a table,
        # minutes later, and the last one is ready long before then.
        RegisterEventHandler(OnProcessExit(
            target_action=arm_gate,
            on_exit=[TimerAction(period=float(i * ARM_STAGGER), actions=[a])
                     for i, a in enumerate(arm_actions)] + [rviz])),
    ])
