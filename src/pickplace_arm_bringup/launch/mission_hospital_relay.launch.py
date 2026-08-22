import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Only the ROS-free layout module: it is the one source of truth for the props,
# the fleet and who runs which leg, and importing the mission module instead
# would drag rclpy, pymoveit2 and a MoveIt config into the launch process to
# learn two namespaces.
from pickplace_arm_bringup.hospital_aws_layout import (
    BENCH_COLLECT, BENCH_DELIVER, DOCK, NAV_ROBOTS, PROP_SPAWN_Z, RACK,
    RELAY_CARRIER_NS, RELAY_RECEIVER_NS, ROBOTS,
)

CARRIER_NS = RELAY_CARRIER_NS
RECEIVER_NS = RELAY_RECEIVER_NS


def generate_launch_description():
    """STAGE 5: the sample run split across two robots.

    mission_hospital_aws.launch.py with a second mission node. The fleet, the
    props and the readiness gates are unchanged in kind - what changes is that
    there are now two of the gate chains, one per robot, and each robot's
    mission node starts behind its own.

    WHY EACH ROBOT NEEDS ITS OWN GATE CHAIN AND NOT A SHARED ONE. The gates
    wait on namespaced things - r1/base_link's TF, /r1/amcl_pose,
    /r1/navigate_to_pose - and the two robots reach those minutes apart: the
    fleet launch deliberately staggers their Nav2 bring-ups against each other
    (NAV_STAGGER, and the change_state gate before it) because starting both at
    once was what killed roughly half of all runs. A single chain gated on r1
    would start r2's mission node while r2 had no navigate_to_pose server at
    all, and the handshake would look like a robot that never came.

    THE TWO NODES DO NOT NEED TO START TOGETHER. That is the point of the
    latched handshake in handover.py: the carrier can be most of the way
    through its collect leg before the receiver's node exists, and the receiver
    still learns the carrier's state the instant it subscribes. So each chain
    runs at its own pace and neither waits on the other.

    THE STARTUP DETACH HAS TO REACH EVERY ROBOT, AND NOW IT DOES. The
    DetachableJoint plugin creates its joint the moment the child model appears,
    without anybody asking - so the instant sample_rack spawns it is welded to
    EVERY robot carrying that plugin, wherever it is standing. A mission node
    publishes its own detach when it starts (see PickAndPlace.__init__), which
    covers r1 and r2 here. Nothing covered r3 and r4: the single-robot launch
    documents that gap in its own docstring and then does not close it, which
    has been harmless only because those two never move. They still never move,
    but a rack welded to a parked robot is a rack pinned to the floor 20 m from
    the bench the moment the plugin's joint takes hold, so the detach is
    published for them here rather than left to luck.
    """
    desc_share = get_package_share_directory('pickplace_arm_description')
    bringup_share = get_package_share_directory('pickplace_arm_bringup')
    models = os.path.join(desc_share, 'models')
    sim = {'use_sim_time': True}

    fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch',
                         'aws_hospital_fleet.launch.py')))

    def spawn(model, name, x, y, z, yaw=0.0):
        return Node(package='ros_gz_sim', executable='create', output='screen',
                    arguments=['-world', 'aws_hospital',
                               '-file', os.path.join(models, model, 'model.sdf'),
                               '-name', name, '-x', str(x), '-y', str(y),
                               '-z', str(z), '-Y', str(yaw)])

    # Poses come from hospital_aws_layout, the same table aws_hospital_map.py
    # stamps into the map. They must not be duplicated here: a bench spawned
    # somewhere the map does not have one is exactly the bug that wedged the
    # robot against thin air.
    rack = spawn('sample_rack', 'sample_rack', RACK[0], RACK[1],
                 PROP_SPAWN_Z['sample_rack'])
    props = [
        spawn('lab_bench', 'bench_collect', BENCH_COLLECT[0], BENCH_COLLECT[1],
              PROP_SPAWN_Z['lab_bench'], BENCH_COLLECT[2]),
        spawn('lab_bench', 'bench_deliver', BENCH_DELIVER[0], BENCH_DELIVER[1],
              PROP_SPAWN_Z['lab_bench'], BENCH_DELIVER[2]),
        rack,
        spawn('delivery_dock', 'delivery_dock', DOCK[0], DOCK[1],
              PROP_SPAWN_Z['delivery_dock']),
    ]

    # The parked robots' startup detach. `ros2 topic pub` rather than a node,
    # because there is nothing to keep alive: three repeats through the bridge
    # and the joint is gone. -w 1 waits for the bridge to subscribe first, which
    # matters for the same reason _publish_box_cmd waits in the mission node -
    # these are one-shot commands with no retry and no acknowledgement, and one
    # sent before discovery completes is simply lost.
    #
    # CHAINED OFF THE RACK'S OWN SPAWN, NOT SENT AT LAUNCH TIME. The joint does
    # not exist until the child model does, so a detach published before
    # sample_rack is inserted is addressed at nothing: it is consumed, the rack
    # then spawns, and the plugin welds it to a robot that has already had its
    # one chance to say no. Waiting for the bridge is not enough on its own -
    # the bridge is up long before the world has finished loading 175 models.
    # The mission nodes never hit this because their gates put them 75 s or more
    # behind the spawn; these have nothing holding them back, so the spawn's own
    # exit is what holds them. The extra few seconds cover the gap between the
    # create service returning and the plugin's next update building the joint.
    mission_ns = (CARRIER_NS, RECEIVER_NS)
    detaches = RegisterEventHandler(OnProcessExit(target_action=rack, on_exit=[
        TimerAction(period=5.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '-t', '3', '-w', '1',
                     f'/{ns}/sample_rack/detach', 'std_msgs/msg/Empty', '{}'],
                output='screen')
            for ns, *_ in ROBOTS if ns not in mission_ns
        ])]))

    # READINESS GATES, and this launch needs them more than the single-robot
    # missions do. The fleet holds each robot's Nav2 stack back until its
    # controllers are up, which is about 75 s in - so a mission node that starts
    # with everything else gets to "START" while AMCL does not exist yet and
    # dies on "no map->base_link TF". Measured, not guessed: that is exactly
    # what the first run did.
    #
    # The frames and actions are namespaced, because each mission runs inside
    # its robot's namespace. `map` is the exception - it is the one frame the
    # whole fleet shares.
    def gate(ns, label, timeout, *args):
        return Node(package='pickplace_arm_bringup', executable='wait_for',
                    name=f'wait_for_{label}', namespace=ns, output='screen',
                    parameters=[sim],
                    arguments=['--label', f'{ns} {label}',
                               '--timeout', str(timeout), *args])

    # TIMEOUTS ARE DELIBERATELY GENEROUS, because a gate that gives up does not
    # stop anything: wait_for logs "Continuing anyway" and exits 0, and the
    # launch chains on OnProcessExit, which fires whatever the exit code. A
    # short timeout therefore does not fail the run - it starts the mission
    # unlocalised, and the mission dies one step later on "no map->base_link TF
    # -- is AMCL running?", which reads like an AMCL bug and is a gate bug.
    #
    # Longer here than in the single-robot launch, and they have to be: the two
    # Nav2 stacks are brought up one after the other on purpose, so the second
    # robot's servers legitimately appear a couple of minutes after the first
    # robot's. A timeout tuned for going first fails the robot that goes second
    # every time, which is the same shape of bug the fleet launch spent three
    # runs chasing before it stopped timing things and started waiting on them.
    actions = []
    for ns, role in ((CARRIER_NS, 'carrier'), (RECEIVER_NS, 'receiver')):
        if ns not in NAV_ROBOTS:
            raise RuntimeError(
                f'{ns} is the relay {role} but is not in NAV_ROBOTS - it has '
                f'no Nav2 stack to drive with. See aws_hospital_fleet.launch.py')
        gate_world = gate(ns, 'world', 300.0, '--clock-stable', '0.5',
                          '--tf', f'{ns}/odom', f'{ns}/base_link')
        gate_amcl = gate(ns, 'amcl', 900.0, '--tf', 'map', f'{ns}/base_link',
                         '--topic', f'/{ns}/amcl_pose')
        gate_nav2 = gate(ns, 'nav2', 900.0,
                         '--action', f'/{ns}/navigate_to_pose',
                         '--action', f'/{ns}/move_action')
        mission = Node(
            package='pickplace_arm_bringup', executable=f'relay_{role}',
            namespace=ns, output='screen', parameters=[sim])
        actions += [
            gate_world,
            RegisterEventHandler(OnProcessExit(target_action=gate_world,
                                               on_exit=[gate_amcl])),
            RegisterEventHandler(OnProcessExit(target_action=gate_amcl,
                                               on_exit=[gate_nav2])),
            RegisterEventHandler(OnProcessExit(target_action=gate_nav2,
                                               on_exit=[mission])),
        ]

    return LaunchDescription([fleet, *props, detaches, *actions])
