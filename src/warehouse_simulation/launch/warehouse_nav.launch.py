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
    nav2_params_path = os.path.join(simulation_pkg, 'config', 'nav2_params.yaml')
    map_yaml_path = os.path.join(
        os.path.expanduser('~'), 'warehouse_robot_ros2', 'maps', 'warehouse_map.yaml'
    )

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

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_path, 'use_sim_time': True}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_path],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_path],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_path],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_path],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_path],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_path],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_path],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_path],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'smoother_server',
                'bt_navigator',
                'velocity_smoother',
                'waypoint_follower',
            ]
        }],
    )

    initial_pose_pub = TimerAction(
        period=8.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/initialpose',
                 'geometry_msgs/msg/PoseWithCovarianceStamped',
                 '{header: {frame_id: "map"}, pose: {pose: {position: {x: -5.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}'],
            output='screen'
        )]
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        lidar_frame_bridge,
        camera_frame_bridge,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        smoother_server,
        bt_navigator,
        velocity_smoother,
        waypoint_follower,
        lifecycle_manager,
        initial_pose_pub,
    ])