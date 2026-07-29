import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """SLAM mapping session: Gazebo (warehouse) + slam_toolbox + RViz.

    Drive the robot around to build the map, then save it:
      ros2 run pickplace_arm_bringup teleop_key        # in a 2nd terminal
      ros2 run nav2_map_server map_saver_cli -f \
          $(ros2 pkg prefix pickplace_arm_bringup)/share/pickplace_arm_bringup/maps/tugbot_warehouse
    (or save into the source tree: .../src/pickplace_arm_bringup/maps/tugbot_warehouse)
    """
    desc_share = get_package_share_directory('pickplace_arm_description')
    bringup_share = get_package_share_directory('pickplace_arm_bringup')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'gazebo.launch.py')))

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'slam.launch.py')))

    # config/navigation.rviz, not the moveit_config one: this session runs no
    # move_group, so a MoveIt MotionPlanning display would block 10 s on an
    # SRDF that never arrives and then log "Robot model not loaded". The
    # navigation config drops that display and keeps what mapping actually
    # needs -- the occupancy grid, the LIDAR scan, TF and the robot model.
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(bringup_share, 'config',
                                      'navigation.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([gazebo, slam, rviz])
