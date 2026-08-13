import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    description_pkg = get_package_share_directory('warehouse_description')
    simulation_pkg = get_package_share_directory('warehouse_simulation')

    urdf_path = os.path.join(description_pkg, 'urdf', 'warehouse_robot.urdf')
    world_path = os.path.join(simulation_pkg, 'worlds', 'warehouse.sdf')
    slam_params_path = os.path.join(simulation_pkg, 'config', 'slam_params.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'warehouse_robot',
            '-topic', 'robot_description',
            '-x', '-5.0',
            '-y', '1.0',
            '-z', '0.2',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

        ],
        output='screen',
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_path],
    )

    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.13', '0', '0', '0', 'base_link', 'warehouse_robot/base_link/lidar'],
    )

    camera_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.21', '0', '0.05', '0', '0', '0', 'base_link', 'warehouse_robot/base_link/rgbd_camera'],
    )

    configure_slam = TimerAction(
        period=4.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
            output='screen'
        )]
    )

    activate_slam = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
            output='screen'
        )]
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        slam_toolbox,
        lidar_frame_bridge,
        camera_frame_bridge,
        configure_slam,
        activate_slam,
    ])