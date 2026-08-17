import json
import py_trees
from py_trees.common import Status
from std_msgs.msg import String


PACKAGE_CONFIDENCE_THRESHOLD = 0.75


class PackageDetected(py_trees.behaviour.Behaviour):
    """
    A py_trees condition behaviour that checks the latest /detections
    message for a 'package' class above the confidence threshold.

    This is a one-shot check (SUCCESS/FAILURE immediately, never RUNNING)
    so it can be composed inside retry loops by parent Selectors.
    """

    def __init__(self, name, node):
        super().__init__(name)
        self._node = node
        self._latest_detections = []
        self._sub = None

    def setup(self, **kwargs):
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

    def update(self):
        for det in self._latest_detections:
            if det.get('class_name') == 'package' and \
                    det.get('confidence', 0.0) >= PACKAGE_CONFIDENCE_THRESHOLD:
                self._node.get_logger().info(
                    f"[{self.name}] Package detected (confidence={det.get('confidence'):.2f})"
                )
                return Status.SUCCESS

        return Status.FAILURE
