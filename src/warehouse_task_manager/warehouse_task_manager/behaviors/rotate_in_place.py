import py_trees
from py_trees.common import Status
from rclpy.action import ActionClient
from nav2_msgs.action import Spin
import math


class RotateInPlace(py_trees.behaviour.Behaviour):
    """
    A py_trees action behaviour that uses Nav2's own Spin recovery
    behavior to rotate the robot in place. This coordinates properly
    with the rest of the Nav2 stack instead of racing other /cmd_vel
    publishers.
    """

    def __init__(self, name, node, spin_radians=1.57):
        super().__init__(name)
        self._node = node
        self._spin_radians = spin_radians
        self._client = None
        self._goal_handle = None
        self._result_future = None
        self._send_goal_future = None
        self._state = "idle"

    def setup(self, **kwargs):
        self._client = ActionClient(self._node, Spin, 'spin')
        return True

    def initialise(self):
        self._state = "sending"
        self._goal_handle = None
        self._result_future = None

        goal = Spin.Goal()
        goal.target_yaw = self._spin_radians

        self._node.get_logger().info(f"[{self.name}] Spinning {math.degrees(self._spin_radians):.0f} degrees")
        self._send_goal_future = self._client.send_goal_async(goal)
        self._state = "waiting_accept"

    def update(self):
        if self._state == "waiting_accept":
            if not self._send_goal_future.done():
                return Status.RUNNING
            self._goal_handle = self._send_goal_future.result()
            if not self._goal_handle.accepted:
                self._node.get_logger().error(f"[{self.name}] Spin goal rejected")
                return Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()
            self._state = "executing"
            return Status.RUNNING

        if self._state == "executing":
            if not self._result_future.done():
                return Status.RUNNING
            result = self._result_future.result()
            self._state = "done"
            self._node.get_logger().info(f"[{self.name}] Spin result status: {result.status}")
            if result.status == 4:  # SUCCEEDED
                return Status.SUCCESS
            else:
                self._node.get_logger().warn(f"[{self.name}] Spin failed (status={result.status})")
                return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status):
        if new_status == Status.INVALID and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
