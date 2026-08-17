import json
import math
import py_trees
from py_trees.common import Status
from rclpy.action import ActionClient
from nav2_msgs.action import Spin
from std_msgs.msg import String


PACKAGE_CONFIDENCE_THRESHOLD = 0.75
STEP_DEGREES = 30
MAX_TOTAL_DEGREES = 360


class ScanForPackage(py_trees.behaviour.Behaviour):
    """
    Rotates the robot in small increments, checking /detections after
    each step, until the package is found or a full rotation has been
    completed without success.
    """

    def __init__(self, name, node):
        super().__init__(name)
        self._node = node
        self._client = None
        self._sub = None
        self._latest_detections = []

        self._goal_handle = None
        self._result_future = None
        self._send_goal_future = None
        self._state = "idle"
        self._degrees_rotated = 0

    def setup(self, **kwargs):
        self._client = ActionClient(self._node, Spin, 'spin')
        self._sub = self._node.create_subscription(
            String, '/detections', self._detections_cb, 10
        )
        return True

    def _detections_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            self._latest_detections = payload.get('detections', [])
        except (json.JSONDecodeError, AttributeError):
            self._latest_detections = []

    def _package_visible(self):
        for det in self._latest_detections:
            if det.get('class_name') == 'package' and \
                    det.get('confidence', 0.0) >= PACKAGE_CONFIDENCE_THRESHOLD:
                return True, det.get('confidence', 0.0)
        return False, 0.0

    def initialise(self):
        self._degrees_rotated = 0
        self._state = "checking"

    def _start_next_spin_step(self):
        goal = Spin.Goal()
        goal.target_yaw = math.radians(STEP_DEGREES)
        self._node.get_logger().info(
            f"[{self.name}] Rotated {self._degrees_rotated} deg so far, stepping {STEP_DEGREES} more"
        )
        self._send_goal_future = self._client.send_goal_async(goal)
        self._state = "waiting_accept"

    def update(self):
        if self._state == "checking":
            found, confidence = self._package_visible()
            if found:
                self._node.get_logger().info(
                    f"[{self.name}] Package found (confidence={confidence:.2f}) "
                    f"after {self._degrees_rotated} deg"
                )
                return Status.SUCCESS

            if self._degrees_rotated >= MAX_TOTAL_DEGREES:
                self._node.get_logger().warn(
                    f"[{self.name}] Completed full rotation, package not found"
                )
                return Status.FAILURE

            self._start_next_spin_step()
            return Status.RUNNING

        if self._state == "waiting_accept":
            if not self._send_goal_future.done():
                return Status.RUNNING
            self._goal_handle = self._send_goal_future.result()
            if not self._goal_handle.accepted:
                self._node.get_logger().error(f"[{self.name}] Spin step rejected")
                return Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()
            self._state = "executing"
            return Status.RUNNING

        if self._state == "executing":
            if not self._result_future.done():
                return Status.RUNNING
            self._degrees_rotated += STEP_DEGREES
            self._state = "checking"
            return Status.RUNNING

        return Status.RUNNING

    def terminate(self, new_status):
        if new_status == Status.INVALID and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
