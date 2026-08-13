import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    simulation_pkg = get_package_share_directory('warehouse_simulation')

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_pkg, 'launch', 'warehouse_nav.launch.py')
        )
    )

    yolo_detector = Node(
        package='warehouse_perception',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    task_manager = Node(
        package='warehouse_task_manager',
        executable='task_manager_node',
        name='warehouse_task_manager',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        nav_launch,
        yolo_detector,
        task_manager,
    ])
