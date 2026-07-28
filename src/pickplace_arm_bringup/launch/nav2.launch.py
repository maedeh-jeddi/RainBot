import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Nav2 navigation stack for the mobile base, plus twist_mux.

    Runs the core Nav2 servers (controller, planner, behaviors, bt_navigator,
    smoother) driven by nav2_params.yaml. The map comes from slam_toolbox
    (run slam.launch.py concurrently), so no map_server/AMCL is launched.

    cmd_vel routing: controller_server publishes geometry_msgs/Twist on
    /cmd_vel. twist_mux merges that (low priority) with the pick node's
    /cmd_vel_search (high priority) and its output is remapped to the diff
    drive controller's unstamped command topic.
    """
    bringup_share = get_package_share_directory('pickplace_arm_bringup')
    nav2_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_time = {'use_sim_time': use_sim_time}

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    nav2_nodes = [
        # controller_server publishes geometry_msgs/Twist on 'cmd_vel'; route
        # it straight to the diff drive controller. (A twist_mux is not used:
        # navigation and the pick node's visual servo run in separate, non-
        # overlapping phases, so no live arbitration is needed -- and the
        # distributed twist_mux binary is unusable on this system, its
        # libdiagnostic_updater.so dependency being absent.)
        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=[nav2_params, sim_time],
             remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')]),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen',
             parameters=[nav2_params, sim_time]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[nav2_params, sim_time]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[nav2_params, sim_time]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[nav2_params, sim_time]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[sim_time, {'autostart': True,
                                    'node_names': lifecycle_nodes}]),
    ]

    return LaunchDescription(
        [DeclareLaunchArgument('use_sim_time', default_value='true')]
        + nav2_nodes)
