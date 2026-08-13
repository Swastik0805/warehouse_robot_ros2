#!/usr/bin/env python3
import math
import yaml
import json
import os
import rclpy.task

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory

from warehouse_interfaces.action import PickupDeliver


PACKAGE_CONFIDENCE_THRESHOLD = 0.5
DETECTION_WAIT_TIMEOUT_SEC = 30.0

SCAN_ANGULAR_SPEED = 0.4       # rad/s while rotating to search
SCAN_STEP_DURATION = 0.5       # seconds of rotation per step
SCAN_SETTLE_DURATION = 0.3     # pause after each step so camera/YOLO can catch up
SCAN_MAX_ROTATION = 2 * math.pi + 0.4  # a bit over a full circle, safety margin


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class TaskManager(Node):
    def __init__(self):
        super().__init__(
            'warehouse_task_manager',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )

        self._locations = self._load_locations()

        self._latest_detections = []
        self._detections_sub = self.create_subscription(
            String, '/detections', self._detections_cb, 10
        )

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._action_server = ActionServer(
            self,
            PickupDeliver,
            'pickup_deliver',
            execute_callback=self._execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f"Task manager ready. Known locations: {list(self._locations.keys())}"
        )

    def _load_locations(self):
        pkg = get_package_share_directory('warehouse_simulation')
        yaml_path = os.path.join(pkg, 'config', 'locations.yaml')
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        return data['locations']

    def _detections_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            self._latest_detections = payload.get('detections', [])
        except (json.JSONDecodeError, AttributeError):
            self._latest_detections = []

    def _check_detections(self):
        for det in self._latest_detections:
            if det.get('class_name') == 'package' and \
                    det.get('confidence', 0.0) >= PACKAGE_CONFIDENCE_THRESHOLD:
                return True, det.get('confidence', 0.0)
        return False, 0.0

    def _publish_angular(self, angular_z):
        twist = Twist()
        twist.angular.z = angular_z
        self._cmd_vel_pub.publish(twist)

    def _build_nav_goal(self, location_name):
        if location_name not in self._locations:
            return None
        loc = self._locations[location_name]
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
        return goal

    async def _navigate_to(self, location_name, goal_handle, step_label, progress):
        nav_goal = self._build_nav_goal(location_name)
        if nav_goal is None:
            return False, f"Unknown location '{location_name}'"

        feedback = PickupDeliver.Feedback()
        feedback.current_step = step_label
        feedback.progress = progress
        goal_handle.publish_feedback(feedback)

        self._nav_client.wait_for_server()
        send_future = self._nav_client.send_goal_async(nav_goal)
        nav_goal_handle = await send_future

        if not nav_goal_handle.accepted:
            return False, f"Navigation to '{location_name}' was rejected by Nav2"

        result_future = nav_goal_handle.get_result_async()
        result = await result_future

        if result.status != 4:
            return False, f"Navigation to '{location_name}' failed (status={result.status})"

        return True, ""

    async def _rclpy_sleep(self, seconds):
        """Await-friendly sleep that works inside rclpy's coroutine executor
        (rclpy does NOT run a real asyncio event loop, so asyncio.sleep()
        raises 'no running event loop' here). Uses an rclpy Future completed
        by a one-shot timer instead."""
        future = rclpy.task.Future()

        def _on_timer():
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(seconds, _on_timer)
        try:
            await future
        finally:
            timer.cancel()
            self.destroy_timer(timer)

    async def _wait_for_package_detection(self, goal_handle):
        feedback = PickupDeliver.Feedback()
        feedback.current_step = "verifying_package"
        feedback.progress = 0.6
        goal_handle.publish_feedback(feedback)

        # First, check without moving -- maybe it's already in view.
        found, conf = self._check_detections()
        if found:
            return True, conf

        self.get_logger().info(
            "Package not immediately visible, rotating to scan surroundings..."
        )

        rotated = 0.0
        elapsed = 0.0
        while rotated < SCAN_MAX_ROTATION and elapsed < DETECTION_WAIT_TIMEOUT_SEC:
            # Rotate a small step
            self._publish_angular(SCAN_ANGULAR_SPEED)
            await self._rclpy_sleep(SCAN_STEP_DURATION)
            self._publish_angular(0.0)
            rotated += SCAN_ANGULAR_SPEED * SCAN_STEP_DURATION
            elapsed += SCAN_STEP_DURATION

            # Let the robot settle and a fresh camera frame arrive
            await self._rclpy_sleep(SCAN_SETTLE_DURATION)
            elapsed += SCAN_SETTLE_DURATION

            found, conf = self._check_detections()
            if found:
                self._publish_angular(0.0)
                self.get_logger().info(
                    f"Package found while scanning (confidence={conf:.2f})"
                )
                return True, conf

        self._publish_angular(0.0)
        return False, 0.0

    async def _execute_callback(self, goal_handle):
        pickup = goal_handle.request.pickup_location
        delivery = goal_handle.request.delivery_location

        self.get_logger().info(
            f"Starting mission: pickup='{pickup}', delivery='{delivery}'"
        )

        ok, msg = await self._navigate_to(
            pickup, goal_handle, "navigating_to_pickup", 0.2
        )
        if not ok:
            goal_handle.abort()
            result = PickupDeliver.Result()
            result.success = False
            result.message = msg
            return result

        detected, confidence = await self._wait_for_package_detection(goal_handle)
        if not detected:
            goal_handle.abort()
            result = PickupDeliver.Result()
            result.success = False
            result.message = f"No package detected at '{pickup}' within timeout"
            return result

        self.get_logger().info(
            f"Package confirmed at '{pickup}' (confidence={confidence:.2f})"
        )

        ok, msg = await self._navigate_to(
            delivery, goal_handle, "navigating_to_delivery", 0.8
        )
        if not ok:
            goal_handle.abort()
            result = PickupDeliver.Result()
            result.success = False
            result.message = msg
            return result

        feedback = PickupDeliver.Feedback()
        feedback.current_step = "complete"
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = PickupDeliver.Result()
        result.success = True
        result.message = (
            f"Delivered package from '{pickup}' to '{delivery}' "
            f"(detection confidence={confidence:.2f})"
        )
        self.get_logger().info(result.message)
        return result


def main():
    rclpy.init()
    node = TaskManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
