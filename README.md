# Autonomous Warehouse Robot — ROS 2 + Gazebo

An autonomous mobile robot that maps a warehouse, localizes itself, navigates between named locations, detects packages using a custom-trained YOLOv8 model, and executes complete pickup-and-delivery missions — all in simulation, built end-to-end with ROS 2 Jazzy and Gazebo Harmonic.

## Demo

![Warehouse robot demo](demo/warehouse_demo_preview.gif)

[Full video with audio](demo/warehouse_demo.mp4)


The robot receives a mission (e.g. "pick up from Shelf A, deliver to the delivery station"), autonomously navigates to the pickup location, visually confirms the package is present using a custom object detector, drives to the delivery point, and reports mission success with a confidence score — no manual intervention required.

Goal accepted, navigating...
current_step: navigating_to_pickup
current_step: verifying_package
current_step: navigating_to_delivery
current_step: complete
Result: success=true
message: "Delivered package from 'shelf_a' to 'delivery_station' (detection confidence=0.92)"


## System Architecture

| Layer | Responsibility | Key Technologies |
|---|---|---|
| Task/Fleet | Mission orchestration, pickup/delivery logic | Custom ROS 2 Action (PickupDeliver) |
| Autonomy | Task execution, package verification | warehouse_task_manager action server |
| Navigation | SLAM, localization, path planning, obstacle avoidance | Nav2 (AMCL, NavFn, RegulatedPurePursuit) |
| Perception | Object detection | Custom-trained YOLOv8 (fine-tuned) |
| Robot | URDF model, differential drive, sensors | ROS 2 URDF/Xacro, ros_gz_bridge |
| Simulation | Physics, environment, sensor simulation | Gazebo Harmonic |

## What This Project Demonstrates

- Full SLAM pipeline: mapping a warehouse environment with slam_toolbox, saving and loading occupancy grid maps
- Autonomous navigation: AMCL localization + Nav2 path planning and obstacle avoidance across a multi-aisle warehouse with named waypoints (shelf_a, shelf_b, shelf_c, pickup_station, delivery_station)
- Custom computer vision: an end-to-end data pipeline — captured ~270 training images directly from the robot's own camera, manually labeled a subset, and fine-tuned a YOLOv8n model to detect a project-specific object class (mAP50 = 0.995 on validation)
- Custom ROS 2 interfaces: defined and compiled a PickupDeliver.action interface with goal/feedback/result fields for long-running task execution
- Multi-node system integration: a single launch file brings up simulation, full Nav2 stack, perception, and task orchestration together
- Real debugging of a real robotics stack: coordinate frame mismatches, costmap/inflation tuning, AMCL parameter tuning for simulated odometry, local-planner deadlocks at convex corners (resolved by switching from DWB to RegulatedPurePursuit), and executor/coroutine pitfalls in async ROS 2 nodes

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12
- Ultralytics (YOLOv8) for perception

## Repository Structure

warehouse_robot_ros2/
├── src/
│ ├── warehouse_description/ # URDF/Xacro robot model
│ ├── warehouse_simulation/ # Gazebo world, launch files, Nav2 config, named locations
│ ├── warehouse_perception/ # Camera capture, YOLO detector node, fine-tuned model
│ ├── warehouse_interfaces/ # Custom PickupDeliver.action definition
│ └── warehouse_task_manager/ # Mission orchestration action server
├── maps/ # Saved occupancy grid maps
└── dataset/ # Training images, labels, YOLO fine-tuning config


## Running the Full System

One command brings up simulation, navigation, perception, and the task manager together:

```bash
source install/setup.bash
ros2 launch warehouse_simulation warehouse_full_system.launch.py
```

Wait ~15-20 seconds for all nodes to fully activate, then send a mission:

```bash
ros2 action send_goal /pickup_deliver warehouse_interfaces/action/PickupDeliver \
  "{pickup_location: 'shelf_a', delivery_location: 'delivery_station'}" --feedback
```

Available named locations are defined in warehouse_simulation/config/locations.yaml and currently include shelf_a, shelf_b, shelf_c, pickup_station, delivery_station, and home.

## Individual Components

Build a new map:
```bash
ros2 launch warehouse_simulation warehouse_sim.launch.py
# drive the robot with teleop_twist_keyboard while SLAM builds the map
ros2 run nav2_map_server map_saver_cli -f ~/warehouse_robot_ros2/maps/warehouse_map
```

Navigate to a single named location:
```bash
ros2 run warehouse_simulation goto_location.py shelf_a
```

Run perception standalone:
```bash
ros2 run warehouse_perception yolo_detector_node
ros2 topic echo /detections
```

Capture new training images:
```bash
ros2 run warehouse_perception capture_images
```

## Perception Model

The object detector was fine-tuned specifically for this project rather than relying on a generic pretrained model:

1. Captured ~270 images from the robot's live camera feed while driving around the simulated package
2. Labeled a training subset (bounding boxes) using Label Studio
3. Fine-tuned YOLOv8n (yolo detect train) starting from COCO pretrained weights
4. Achieved mAP50 = 0.995, Precision = 0.99, Recall = 1.0 on the validation split
5. Deployed the resulting weights into the live detection node, replacing generic/incorrect detections (e.g. a stock model misclassifying the package as "bed") with accurate, high-confidence detections (~0.85-0.92 confidence) of the actual object class

## Roadmap

This project follows a phased roadmap from ROS 2 fundamentals through to a portfolio-ready autonomous robot. Completed phases:

- [x] Phase 0-1: ROS 2 fundamentals, workspace, core communication patterns
- [x] Phase 2-3: Simulated differential-drive robot, motion control
- [x] Phase 4: Sensor integration (LiDAR, camera)
- [x] Phase 5: SLAM mapping and localization
- [x] Phase 6: Autonomous navigation with Nav2
- [x] Phase 7: Structured warehouse environment with named locations
- [x] Phase 8: Computer vision — custom-trained object detection
- [x] Phase 9: Task manager — complete pickup-and-delivery missions

Planned next: behavior trees for more structured decision-making (Phase 10), deliberate failure-recovery testing (Phase 11), and engineering polish — automated tests, CI, Docker (Phase 13).

## Key Engineering Lessons

- Coordinate frame discipline matters. A silent offset between the SLAM map's origin and the robot's Gazebo spawn point caused a cascade of navigation failures that looked like unrelated bugs until the root cause was isolated.
- Local planners are not interchangeable. DWB's multi-critic trajectory scoring produced a genuine local-minimum deadlock at convex shelf corners; switching to RegulatedPurePursuit's geometric lookahead approach resolved it structurally rather than through further parameter tuning.
- rclpy executors and Python's asyncio don't mix by default. Calling asyncio.sleep() or rclpy.spin_once() from inside an already-spinning multi-threaded executor silently breaks assumptions about the underlying event loop — the fix was a custom Future-based wait helper.
- Source vs. install directory mismatches are a recurring trap in ROS 2 development — several bugs in this project traced back to editing a source file without confirming the change had actually propagated to the installed copy used at runtime.
