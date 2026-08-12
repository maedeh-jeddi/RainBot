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
    robot in it. (0, 7.5) is the middle of the lobby and is the roomiest spot in
    the building: 7.29 m to the nearest wall, measured off the wall collision
    mesh rather than judged by eye.

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
        SetEnvironmentVariable('SPAWN_X', '0.0'),
        SetEnvironmentVariable('SPAWN_Y', '7.5'),
        gazebo,
    ])
