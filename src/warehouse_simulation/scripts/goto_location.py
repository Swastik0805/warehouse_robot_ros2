#!/usr/bin/env python3
import sys
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory
import os


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GotoLocation(Node):
    def __init__(self, location_name):
        super().__init__(
            'goto_location',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        pkg = get_package_share_directory('warehouse_simulation')
        yaml_path = os.path.join(pkg, 'config', 'locations.yaml')

        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        locations = data['locations']
        if location_name not in locations:
            self.get_logger().error(
                f"Unknown location '{location_name}'. Available: {list(locations.keys())}"
            )
            rclpy.shutdown()
            sys.exit(1)

        loc = locations[location_name]
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(loc['x'])
        goal.pose.pose.position.y = float(loc['y'])
        qx, qy, qz, qw = yaw_to_quaternion(float(loc.get('yaw', 0.0)))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Navigating to '{location_name}' at ({loc['x']}, {loc['y']})"
        )
        self._client.wait_for_server()
        self._send_goal(goal)

    def _send_goal(self, goal):
        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {remaining:.2f} m', throttle_duration_sec=2.0)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted, navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().warn(f'Goal finished with status: {status}')
        rclpy.shutdown()


def main():
    if len(sys.argv) < 2:
        print("Usage: goto_location.py <location_name>")
        sys.exit(1)

    rclpy.init()
    node = GotoLocation(sys.argv[1])
    rclpy.spin(node)


if __name__ == '__main__':
    main()
