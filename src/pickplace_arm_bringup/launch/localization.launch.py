import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Localize on the saved map (map_server + AMCL) and run Nav2.

    Gazebo + map_server serving maps/tugbot_warehouse.yaml + AMCL (supplies
    map->odom) + the Nav2 core + RViz. No slam_toolbox. Send goals from RViz
    "Nav2 Goal"; the robot plans, follows, and avoids obstacles.
    """
    desc_share = get_package_share_directory('pickplace_arm_description')
    bringup_share = get_package_share_directory('pickplace_arm_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim = {'use_sim_time': use_sim_time}
    map_yaml = LaunchConfiguration('map')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'gazebo.launch.py')))

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'nav2.launch.py')))

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[sim, {'yaml_filename': map_yaml}])

    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'amcl_tugbot.yaml'), sim])

    localization_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[sim, {'autostart': True,
                          'node_names': ['map_server', 'amcl']}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', os.path.join(
            get_package_share_directory('pickplace_arm_moveit_config'),
            'config', 'moveit.rviz')],
        parameters=[sim])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            bringup_share, 'maps', 'tugbot_warehouse.yaml')),
        gazebo,
        map_server,
        amcl,
        localization_lifecycle,
        nav2,
        rviz,
    ])
