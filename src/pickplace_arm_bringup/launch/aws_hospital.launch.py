import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Bring the AWS hospital up with the robot and two walking pedestrians.

    A place to look at and drive around the big world; the sample transport
    mission is not wired to this building yet.

    SPAWN POSE. The world origin is INSIDE A WALL - spawning there embeds the
    robot in it. (-3.25, 8.5) is in the lobby, north-west of the nurses' station,
    and is the best-balanced clear spot in the building:

        4.37 m to the nearest wall      (wall collision mesh)
        3.06 m to the nurses' station   (its collision mesh, not its origin)
        3.20 m to the nearest prop      (include poses)
        2.50 m to the nearest actor path (waypoint-to-waypoint segments)

    IT USED TO BE (0, 7.5), CHOSEN ON WALL CLEARANCE ALONE - 7.29 m, the roomiest
    spot in the building by that one measure. Then lobby_planter went into the
    world at (0, 7.75). The planter's pot collision has a 0.36 m radius and the
    spawn point sits 0.235 m from its centre, so the robot spawned INSIDE it: it
    climbed the saucer, ended up at z = 0.179 instead of the 0.14 it was given,
    and sat with a 0.069 rad (3.9 deg) roll. A tilted spawn is not cosmetic - the
    IMU and the LIDAR plane start off level.
    Measure a spawn point against the furniture and the actor paths, not just the
    walls.

    THE BUILDING, measured the same way: 27 x 58 m overall, of which 831 m2 is
    floor the Husky actually fits on and can reach from the lobby, with routes
    up to 98 m long.

    PERFORMANCE. This world runs at a real-time factor near 0.5, against 1.0 for
    hospital_lab - it carries 182 models. Nothing is dropped: the LIDAR is
    configured for 10 Hz and measures 4.93 Hz on the wall clock, i.e. exactly
    RTF x 10, so in SIMULATION time it keeps full rate and every node runs on
    use_sim_time. It simply takes about twice as long to watch.
    """
    desc_share = get_package_share_directory('pickplace_arm_description')
    models = os.path.join(desc_share, 'models')
    sim = {'use_sim_time': True}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'gazebo.launch.py')))

    def spawn(model, name, x, y, z, yaw=0.0):
        return Node(package='ros_gz_sim', executable='create', output='screen',
                    arguments=['-world', 'aws_hospital',
                               '-file', os.path.join(models, model, 'model.sdf'),
                               '-name', name, '-x', str(x), '-y', str(y),
                               '-z', str(z), '-Y', str(yaw)])

    # NO pedestrian spawning or patrol node here any more. The walkers are
    # <actor>s declared in aws_hospital.sdf itself, because only actors animate
    # a skeleton - a driven <model> slides rather than walks - and an actor's
    # scripted trajectory needs no external commander. That also removed the
    # failure the model-based pedestrians kept hitting, where one of the two
    # jammed against a wall and stopped: a trajectory cannot jam.
    return LaunchDescription([
        SetEnvironmentVariable('WORLD', 'aws_hospital.sdf'),
        SetEnvironmentVariable('SPAWN_X', '-3.25'),
        SetEnvironmentVariable('SPAWN_Y', '8.5'),
        # 182 models' worth of meshes take the GUI well past the point where the
        # server is ready to accept the robot, and a model inserted before the
        # GUI's scene is built never gets drawn - see gazebo.launch.py's spawn
        # gate. Hold the insert until the viewport has settled.
        SetEnvironmentVariable('SPAWN_DELAY', '45'),
        gazebo,
    ])
