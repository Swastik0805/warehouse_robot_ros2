#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
"""
YOLO-based object detector for the warehouse robot.

Subscribes to the RGB camera feed, runs YOLO inference on each frame,
and publishes:
  - /detections/image  (sensor_msgs/Image)   annotated debug image with boxes drawn
  - /detections         (std_msgs/String)     JSON-encoded list of detections

NOTE: This node intentionally does NOT use cv_bridge. cv_bridge is a
compiled extension shipped via apt that is linked against NumPy 1.x ABI,
while ultralytics/torch/opencv-python require NumPy 2.x. Mixing them
causes segfaults / ImportErrors. Instead we convert sensor_msgs/Image
<-> numpy arrays manually, which only requires numpy (no ABI conflict).
"""

import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

MODEL_PATH = os.path.join(get_package_share_directory("warehouse_perception"), "models", "package_detector.pt")
CONFIDENCE_THRESHOLD = 0.4


def imgmsg_to_numpy(msg):
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == "rgb8":
        arr = arr[:, :, ::-1]
    elif msg.encoding != "bgr8":
        raise ValueError(f"Unsupported encoding: {msg.encoding}")
    return np.ascontiguousarray(arr)


def numpy_to_imgmsg(arr, header):
    msg = Image()
    msg.header = header
    msg.height = arr.shape[0]
    msg.width = arr.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = arr.shape[1] * arr.shape[2]
    msg.data = arr.tobytes()
    return msg


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector_node")
        self.get_logger().info(f"Loading YOLO model: {MODEL_PATH}")
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info("YOLO model loaded.")
        self.image_sub = self.create_subscription(Image, "/camera/image", self.image_callback, 10)
        self.annotated_pub = self.create_publisher(Image, "/detections/image", 10)
        self.detections_pub = self.create_publisher(String, "/detections", 10)
        self.get_logger().info("YOLO detector node ready, subscribed to /camera/image")

    def image_callback(self, msg):
        try:
            cv_image = imgmsg_to_numpy(msg)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        results = self.model.predict(source=cv_image, conf=CONFIDENCE_THRESHOLD, verbose=False)
        detections = []
        result = results[0]

        for box in result.boxes:
            cls_id = int(box.cls[0])
            class_name = self.model.names[cls_id]
            confidence = float(box.conf[0])
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            detections.append({
                "class_name": class_name,
                "confidence": round(confidence, 3),
                "bbox": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)]
            })

        detections_msg = String()
        detections_msg.data = json.dumps({
            "header_stamp_sec": msg.header.stamp.sec,
            "header_stamp_nanosec": msg.header.stamp.nanosec,
            "detections": detections
        })
        self.detections_pub.publish(detections_msg)

        annotated = result.plot()
        annotated_msg = numpy_to_imgmsg(annotated, msg.header)
        self.annotated_pub.publish(annotated_msg)

        if detections:
            self.get_logger().info(
                f"Detected {len(detections)} object(s): " + ", ".join(f"{d['class_name']} ({d['confidence']})" for d in detections),
                throttle_duration_sec=2.0
            )


def main():
    rclpy.init()
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
