import py_trees
from py_trees.common import Status
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class NavigateTo(py_trees.behaviour.Behaviour):
    """
    A py_trees behaviour that sends a NavigateToPose goal to Nav2 and
    reports RUNNING while in progress, SUCCESS/FAILURE on completion.
    """

    def __init__(self, name, node, location_name, locations):
        super().__init__(name)
        self._node = node
        self._location_name = location_name
        self._locations = locations
        self._client = None
        self._goal_handle = None
        self._result_future = None
        self._send_goal_future = None
        self._state = "idle"  # idle -> sending -> waiting_accept -> executing -> done

    def setup(self, **kwargs):
        self._client = ActionClient(self._node, NavigateToPose, 'navigate_to_pose')
        return True

    def initialise(self):
        self._state = "sending"
        self._goal_handle = None
        self._result_future = None
        self._send_goal_future = None

        loc = self._locations.get(self._location_name)
        if loc is None:
            self._state = "error"
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(loc['x'])
        goal.pose.pose.position.y = float(loc['y'])
        qx, qy, qz, qw = yaw_to_quaternion(float(loc.get('yaw', 0.0)))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._node.get_logger().info(f"[{self.name}] Navigating to '{self._location_name}'")
        self._send_goal_future = self._client.send_goal_async(goal)
        self._state = "waiting_accept"

    def update(self):
        if self._state == "error":
            self._node.get_logger().error(f"[{self.name}] Unknown location '{self._location_name}'")
            return Status.FAILURE

        if self._state == "waiting_accept":
            if not self._send_goal_future.done():
                return Status.RUNNING
            self._goal_handle = self._send_goal_future.result()
            if not self._goal_handle.accepted:
                self._node.get_logger().error(f"[{self.name}] Goal rejected")
                return Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()
            self._state = "executing"
            return Status.RUNNING

        if self._state == "executing":
            if not self._result_future.done():
                return Status.RUNNING
            result = self._result_future.result()
            self._state = "done"
            if result.status == 4:  # SUCCEEDED
                self._node.get_logger().info(f"[{self.name}] Reached '{self._location_name}'")
                return Status.SUCCESS
            else:
                self._node.get_logger().warn(
                    f"[{self.name}] Navigation to '{self._location_name}' failed (status={result.status})"
                )
                return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status):
        if new_status == Status.INVALID and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
