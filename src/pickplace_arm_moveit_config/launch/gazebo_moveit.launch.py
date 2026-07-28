import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Full pick-and-place bringup: Gazebo Harmonic (real ros2_control
    controllers) + MoveIt move_group + RViz motion-planning UI.

    Unlike demo.launch.py (which spins up its own mock ros2_control_node),
    this reuses the controllers hosted by gz_ros2_control inside Gazebo, so
    plans executed from RViz actually move the arm in the simulation.
    """
    moveit_config = (
        MoveItConfigsBuilder("pickplace_arm", package_name="pickplace_arm_moveit_config")
        .to_moveit_configs()
    )

    use_sim_time = {"use_sim_time": True}

    # 1) Gazebo Harmonic + robot_state_publisher + gz_ros2_control controllers
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("pickplace_arm_description"),
                "launch",
                "gazebo.launch.py",
            )
        )
    )

    # 2) MoveIt move_group (planning), driven off simulation time
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            use_sim_time,
            # Tolerate small joint-state settling drift between rapid arm moves
            # (else MoveIt aborts with "start point deviates from current robot
            # state"; default tolerance is a tight 0.01 rad).
            {"trajectory_execution.allowed_start_tolerance": 0.1},
        ],
    )

    # 3) RViz with the MoveIt MotionPlanning display
    rviz_config = os.path.join(
        get_package_share_directory("pickplace_arm_moveit_config"),
        "config",
        "moveit.rviz",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            use_sim_time,
        ],
    )

    return LaunchDescription([gazebo, move_group, rviz])
