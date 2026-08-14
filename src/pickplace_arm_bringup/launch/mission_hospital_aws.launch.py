import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from pickplace_arm_bringup.hospital_aws_layout import (
    BENCH_COLLECT, BENCH_DELIVER, DOCK, PROP_SPAWN_Z, RACK,
)

# Which robot runs the single-robot version of the route. Must be one of
# ARM_ROBOTS in aws_hospital_fleet.launch.py, or there is no move_group to plan
# with and no navigate_to_pose to drive with.
MISSION_ROBOT = 'r1'


def generate_launch_description():
    """The sample run in the AWS hospital, on top of the standing fleet.

    Brings up aws_hospital_fleet.launch.py unchanged - four robots in their
    fixed ring, two of them navigating and armed - then adds what the mission
    needs: two benches, the rack, the dock, and the mission node itself in the
    robot's namespace.

    THE PROPS ARE SPAWNED, NOT PUT IN THE WORLD FILE. aws_hospital.sdf is the
    building; a rack that gets carried away is mission payload. Keeping it out
    of the world also keeps it out of maps/aws_hospital.yaml, which matters
    because that map is generated from the world's own geometry - a bench baked
    into the world would become permanent occupied cells that Nav2 plans around
    even after the bench is gone.

    SPAWN Z IS THE MODEL ORIGIN. lab_bench sits on the floor at 0; the rack and
    the dock both carry their origin on their own underside, so each spawns at
    the height of the shelf it stands on, 0.30.

    THE STARTUP DETACH IS LOAD-BEARING AND MUST REACH EVERY ROBOT. The
    DetachableJoint plugin creates its joint the instant the child model
    appears, without anybody asking - so the moment sample_rack spawns it is
    welded to EVERY robot carrying that plugin, wherever they are standing. With
    one robot that cost a single detach at startup; with a fleet it is one per
    robot, and skipping any of them tows the rack off the bench sideways as soon
    as that robot moves. The mission node publishes its own; the others are
    published here.
    """
    desc_share = get_package_share_directory('pickplace_arm_description')
    models = os.path.join(desc_share, 'models')
    sim = {'use_sim_time': True}

    fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pickplace_arm_bringup'),
                         'launch', 'aws_hospital_fleet.launch.py')))

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
    props = [
        spawn('lab_bench', 'bench_collect', BENCH_COLLECT[0], BENCH_COLLECT[1],
              PROP_SPAWN_Z['lab_bench'], BENCH_COLLECT[2]),
        spawn('lab_bench', 'bench_deliver', BENCH_DELIVER[0], BENCH_DELIVER[1],
              PROP_SPAWN_Z['lab_bench'], BENCH_DELIVER[2]),
        spawn('sample_rack', 'sample_rack', RACK[0], RACK[1],
              PROP_SPAWN_Z['sample_rack']),
        spawn('delivery_dock', 'delivery_dock', DOCK[0], DOCK[1],
              PROP_SPAWN_Z['delivery_dock']),
    ]

    # READINESS GATES, and this launch needs them more than the single-robot
    # missions do. The fleet holds each robot's Nav2 stack back until its
    # controllers are up, which is about 75 s in - so a mission node that starts
    # with everything else gets to "MISSION 2: START" while AMCL does not exist
    # yet and dies on "no map->base_link TF". Measured, not guessed: that is
    # exactly what the first run did.
    #
    # The frames and actions are namespaced, because the mission runs inside the
    # robot's namespace. `map` is the exception - it is the one frame the whole
    # fleet shares.
    ns = MISSION_ROBOT

    def gate(label, timeout, *args):
        return Node(package='pickplace_arm_bringup', executable='wait_for',
                    name=f'wait_for_{label}', output='screen', parameters=[sim],
                    arguments=['--label', label, '--timeout', str(timeout), *args])

    # TIMEOUTS ARE DELIBERATELY GENEROUS, because a gate that gives up does not
    # stop anything: wait_for logs "Continuing anyway" and exits 0, and the
    # launch chains on OnProcessExit, which fires whatever the exit code. A
    # short timeout therefore does not fail the run - it starts the mission
    # unlocalised, and the mission dies one step later on "no map->base_link TF
    # -- is AMCL running?", which reads like an AMCL bug and is a gate bug.
    #
    # That is not hypothetical: Nav2's lifecycle bringup sat on "Configuring
    # controller_server" for about five minutes on a loaded run, then came up
    # perfectly well. It was slow, not broken. A gate exits the moment its
    # condition is met, so waiting longer costs nothing when the machine is
    # quick and saves the whole run when it is not.
    gate_world = gate('world', 300.0, '--clock-stable', '0.5',
                      '--tf', f'{ns}/odom', f'{ns}/base_link')
    gate_amcl = gate('amcl', 600.0, '--tf', 'map', f'{ns}/base_link',
                     '--topic', f'/{ns}/amcl_pose')
    gate_nav2 = gate('nav2', 600.0,
                     '--action', f'/{ns}/navigate_to_pose',
                     '--action', f'/{ns}/move_action')

    mission = Node(
        package='pickplace_arm_bringup', executable='mission_hospital_aws',
        namespace=ns, output='screen', parameters=[sim])

    return LaunchDescription([
        fleet, *props,
        gate_world,
        RegisterEventHandler(OnProcessExit(target_action=gate_world,
                                           on_exit=[gate_amcl])),
        RegisterEventHandler(OnProcessExit(target_action=gate_amcl,
                                           on_exit=[gate_nav2])),
        RegisterEventHandler(OnProcessExit(target_action=gate_nav2,
                                           on_exit=[mission])),
    ])
