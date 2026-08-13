#!/usr/bin/env python3
"""
Captures frames from the robot's camera topic at a fixed interval and saves
them to disk for building a YOLO fine-tuning dataset.

Usage:
    ros2 run warehouse_perception capture_images.py [output_dir] [interval_sec]

Defaults:
    output_dir   = ~/warehouse_robot_ros2/dataset/raw_images
    interval_sec = 1.0
"""
import sys
import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def imgmsg_to_numpy(msg):
    dtype = np.uint8
    channels = 3 if msg.encoding in ('rgb8', 'bgr8') else 1
    img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
    if msg.encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


class ImageCapture(Node):
    def __init__(self, output_dir, interval):
        super().__init__('image_capture')
        self.output_dir = output_dir
        self.interval = interval
        self.last_save_time = 0.0
        self.count = 0

        os.makedirs(self.output_dir, exist_ok=True)

        self.sub = self.create_subscription(
            Image, '/camera/image', self.callback, 10
        )
        self.get_logger().info(
            f"Capturing to '{self.output_dir}' every {self.interval}s. "
            f"Drive the robot around your package(s) now."
        )

    def callback(self, msg):
        now = time.time()
        if now - self.last_save_time < self.interval:
            return
        self.last_save_time = now

        img = imgmsg_to_numpy(msg)
        filename = os.path.join(self.output_dir, f'frame_{self.count:04d}.jpg')
        cv2.imwrite(filename, img)
        self.count += 1
        self.get_logger().info(f'Saved {filename} (total: {self.count})')


def main():
    default_dir = os.path.join(
        os.path.expanduser('~'), 'warehouse_robot_ros2', 'dataset', 'raw_images'
    )
    output_dir = sys.argv[1] if len(sys.argv) > 1 else default_dir
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

    rclpy.init()
    node = ImageCapture(output_dir, interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Stopped. Captured {node.count} images total.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
