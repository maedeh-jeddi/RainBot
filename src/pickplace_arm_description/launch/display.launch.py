import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Description-only view: robot_state_publisher + joint_state_publisher_gui
    + RViz. NO controllers, NO Gazebo -- just the URDF, with GUI sliders to jog
    every joint and watch the model/TF update in RViz.

    The URDF is generated with use_gazebo:=false so the ros2_control system and
    Gazebo sensor plugins are excluded (pure kinematic description).
    """
    pkg_share = get_package_share_directory('pickplace_arm_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'pickplace_arm.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'display.rviz')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' use_gazebo:=false']),
            value_type=str)
    }

    # Strip the VS Code *snap* GTK/GIO/LOCPATH env vars from the Qt GUIs (RViz
    # and joint_state_publisher_gui); otherwise they load a glibc-incompatible
    # snap libpthread and crash (symbol lookup error __libc_pthread_init).
    qt_env_fix = ('env -u GTK_PATH -u GTK_EXE_PREFIX -u LOCPATH '
                  '-u GDK_PIXBUF_MODULE_FILE -u GDK_PIXBUF_MODULEDIR '
                  '-u GIO_MODULE_DIR -u GTK_IM_MODULE_FILE')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        # GUI with a slider per joint -> publishes /joint_states
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            prefix=qt_env_fix,
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            prefix=qt_env_fix,
            arguments=['-d', rviz_config_file],
        ),
    ])
