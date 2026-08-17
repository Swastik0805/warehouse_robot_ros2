#!/usr/bin/env python3
import os
import yaml
import py_trees
from py_trees.common import Status

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory
from warehouse_interfaces.action import PickupDeliver

from warehouse_task_manager.behaviors.navigate_to import NavigateTo
from warehouse_task_manager.behaviors.package_detected import PackageDetected
from warehouse_task_manager.behaviors.scan_for_package import ScanForPackage


def build_mission_tree(node, locations, pickup_location, delivery_location):
    """
    Selector: MissionRoot
    └── Sequence: PickupAndDeliver
        ├── NavigateTo(pickup)
        ├── Selector: VerifyPackage (retry with scan)
        │   ├── PackageDetected
        │   └── Sequence: RotateAndRetry
        │       ├── RotateInPlace
        │       └── PackageDetected
        └── NavigateTo(delivery)
    """
    nav_to_pickup = NavigateTo("NavigateToPickup", node, pickup_location, locations)

    initial_check = PackageDetected("PackageDetected", node)
    scan_for_package = ScanForPackage("ScanForPackage", node)

    verify_package = py_trees.composites.Selector(
        name="VerifyPackage", memory=True
    )
    verify_package.add_children([initial_check, scan_for_package])

    nav_to_delivery = NavigateTo("NavigateToDelivery", node, delivery_location, locations)

    pickup_and_deliver = py_trees.composites.Sequence(
        name="PickupAndDeliver", memory=True
    )
    pickup_and_deliver.add_children([nav_to_pickup, verify_package, nav_to_delivery])

    return py_trees.trees.BehaviourTree(pickup_and_deliver)


class BTTaskManager(Node):
    def __init__(self):
        super().__init__(
            'warehouse_bt_task_manager',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self._locations = self._load_locations()

        self._action_server = ActionServer(
            self,
            PickupDeliver,
            'pickup_deliver_bt',
            execute_callback=self._execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f"BT Task manager ready. Known locations: {list(self._locations.keys())}"
        )

    def _load_locations(self):
        pkg = get_package_share_directory('warehouse_simulation')
        yaml_path = os.path.join(pkg, 'config', 'locations.yaml')
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        return data['locations']

    async def _execute_callback(self, goal_handle):
        pickup = goal_handle.request.pickup_location
        delivery = goal_handle.request.delivery_location

        self.get_logger().info(
            f"Starting BT mission: pickup='{pickup}', delivery='{delivery}'"
        )

        tree = build_mission_tree(self, self._locations, pickup, delivery)
        tree.setup(node=self)

        feedback = PickupDeliver.Feedback()

        rate_hz = 5.0
        while rclpy.ok():
            tree.tick()
            status = tree.root.status

            feedback.current_step = tree.root.tip().name if tree.root.tip() else "ticking"
            feedback.progress = 0.5
            goal_handle.publish_feedback(feedback)

            if status == Status.SUCCESS:
                goal_handle.succeed()
                result = PickupDeliver.Result()
                result.success = True
                result.message = f"Delivered package from '{pickup}' to '{delivery}'"
                self.get_logger().info(result.message)
                return result

            if status == Status.FAILURE:
                goal_handle.abort()
                result = PickupDeliver.Result()
                result.success = False
                result.message = "Mission failed - see logs for which step failed"
                self.get_logger().warn(result.message)
                return result

            rclpy.spin_once(self, timeout_sec=1.0 / rate_hz)


def main():
    rclpy.init()
    node = BTTaskManager()
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
