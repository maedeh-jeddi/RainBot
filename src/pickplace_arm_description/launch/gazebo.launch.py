import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('pickplace_arm_description')

    # Gazebo resolves package://<pkg>/... mesh URIs (as robot_state_publisher
    # emits them) by rewriting the scheme to model://<pkg>/... and searching
    # GZ_SIM_RESOURCE_PATH for a <dir>/<pkg>/... match, so the share dir's
    # PARENT is what goes on the path (get_package_share_directory already
    # returns .../share/<pkg_name>).
    #
    # This used to list four entries -- clearpath_platform_description,
    # franka_description, realsense2_description and sick_scan_xd. All of that
    # geometry now lives in this package's own meshes/ (see CMakeLists), so one
    # entry covers the Husky A200, the FR3 + Franka Hand, and the RealSense/SICK
    # sensor housings alike.
    gz_resource_paths = [
        os.path.dirname(pkg_description),
    ]

    # tugbot_warehouse.sdf pulls the warehouse shell, shelves, pallets, carts
    # and Tugbot in from Gazebo Fuel. Those models are vendored into
    # fuel_cache/ (see CMakeLists.txt) so pointing the Fuel client's cache
    # root there makes it a cache hit instead of a network fetch -- no
    # ~/.gz/fuel pre-fetch needed on a fresh machine.
    fuel_cache_path = os.path.join(pkg_description, 'fuel_cache')

    xacro_file = os.path.join(pkg_description, 'urdf', 'pickplace_arm.urdf.xacro')
    # tugbot_warehouse.sdf is the only world this project ships, and the one
    # the mission's saved map was built against. Override with the WORLD env
    # var to point at another .sdf under worlds/.
    world_name = os.environ.get('WORLD', 'tugbot_warehouse.sdf')
    world_file = os.path.join(pkg_description, 'worlds', world_name)
    # World origin (0,0) is clear in this world and is where the map origin
    # sits. A denser world can have the origin inside furniture, so
    # SPAWN_X/SPAWN_Y let a different world pick a clear spot to spawn into.
    spawn_x = os.environ.get('SPAWN_X', '0.0')
    spawn_y = os.environ.get('SPAWN_Y', '0.0')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' use_gazebo:=true']), value_type=str
        )
    }

    # HEADLESS=1 runs the Gazebo SERVER only (no GUI, `-s`). Needed for heavy
    # worlds: a GUI carrying a GlobalIlluminationVct plugin (high-quality voxel
    # GI, 9 light bounces) was measured dragging the real-time factor
    # down to ~0.28, which starves the LIDAR (drops to ~3 Hz) and makes
    # slam_toolbox's scan matcher fail during rotation -- the map->base_link
    # TF freezes while the robot physically spins. Sensor rendering (RGB-D,
    # LIDAR) happens on the SERVER, so it is unaffected by dropping the GUI.
    gz_flags = '-s -r ' if os.environ.get('HEADLESS') == '1' else '-r '
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        # gz_version 8 = Gazebo Harmonic
        launch_arguments={
            'gz_args': gz_flags + world_file,
            'gz_version': '8',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'pickplace_arm',
            '-x', spawn_x,
            '-y', spawn_y,
            # base_link (the root) is the Husky A200's chassis origin, which
            # sits (wheel_radius - wheel_vertical_offset) = 0.1651 - 0.03282
            # = 0.13228 m above the ground so the wheels touch the floor;
            # base_footprint hangs exactly that far below it. A small margin
            # is added so the robot settles onto the floor rather than
            # spawning interpenetrating it.
            '-z', '0.14'
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )

    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delayed_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delayed_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    delayed_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Simulation clock -> ROS, so use_sim_time nodes get a time source
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Front base-mounted RGB-D camera (box detection while driving)
            '/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/front_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # Rigid-grasp control (ROS -> gz, hence ']'): one attach/detach pair
            # per box, driven by the DetachableJoint plugins on the robot (see
            # pickplace_arm.gazebo.xacro). The gripper welds the box on a
            # VERIFIED grasp and releases it on open; friction alone let the box
            # slip out mid-carry on every Tugbot-warehouse run.
            '/box_red/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/box_red/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/box_green/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/box_green/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/box_blue/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/box_blue/detach@std_msgs/msg/Empty]gz.msgs.Empty',
        ],
        output='screen',
    )

    # robot_localization EKF: fuses wheel odometry (forward velocity) with the
    # IMU (heading) to publish a stable odom -> base_link transform. The
    # diff_drive controller's own odom TF is disabled (enable_odom_tf: false)
    # so this is the single source of that transform.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_description, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ],
    )

    set_gz_resource_path = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', p) for p in gz_resource_paths
    ]
    set_gz_fuel_cache_path = SetEnvironmentVariable('GZ_FUEL_CACHE_PATH', fuel_cache_path)

    return LaunchDescription([
        *set_gz_resource_path,
        set_gz_fuel_cache_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        delayed_gripper_controller,
        delayed_diff_drive_controller,
        bridge,
        ekf,
    ])
