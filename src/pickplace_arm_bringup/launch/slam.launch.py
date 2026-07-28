import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """slam_toolbox online async mapping for the mobile base.

    Run alongside gazebo.launch.py (which provides /scan and the
    odom -> base_link TF from diff_drive_controller). slam_toolbox adds the
    map -> odom transform and builds an occupancy grid on /map.
    """
    bringup_share = get_package_share_directory('pickplace_arm_bringup')
    slam_params = os.path.join(bringup_share, 'config', 'slam_toolbox.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
        ),
    ])
