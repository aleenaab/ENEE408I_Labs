#!/usr/bin/env python3
"""Publish OpenCV camera frames as ROS 2 sensor_msgs/Image messages."""

import math
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Image


def main(args=None):
    rclpy.init(args=args)
    node = None
    capture = None
    exit_code = 0

    try:
        node = Node("camera_publisher")
        # Accept either an integer camera index or a string device path.
        node.declare_parameter(
            "device", 0, ParameterDescriptor(dynamic_typing=True)
        )
        node.declare_parameter("image_topic", "/camera/image_raw")
        node.declare_parameter("frame_id", "camera_optical_frame")
        node.declare_parameter(
            "fps", 15.0, ParameterDescriptor(dynamic_typing=True)
        )

        device = node.get_parameter("device").value
        if isinstance(device, str) and device.isdigit():
            device = int(device)
        if isinstance(device, bool) or not isinstance(device, (int, str)):
            raise ValueError("device must be an integer camera index or device path")
        topic = node.get_parameter("image_topic").value
        frame_id = node.get_parameter("frame_id").value
        fps = float(node.get_parameter("fps").value)
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("fps must be a positive finite number")

        capture = cv2.VideoCapture(device)
        if not capture.isOpened():
            raise RuntimeError(
                "Cannot open camera %r. Check the device, camera permissions, "
                "and whether the camera is accessible inside this Ubuntu environment."
                % device
            )
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        capture.set(cv2.CAP_PROP_FPS, fps)
        bridge = CvBridge()
        publisher = node.create_publisher(Image, topic, 1)
        node.get_logger().info(
            f"Camera opened; publishing on {topic}. Ctrl+C to exit."
        )
        failures = 0
        while rclpy.ok():
            # Service ROS callbacks without blocking camera capture.
            rclpy.spin_once(node, timeout_sec=0.0)
            if not rclpy.ok():
                break
            started = time.monotonic()
            ok, frame = capture.read()
            if not ok or frame is None:
                failures += 1
                node.get_logger().warning(
                    "Camera opened but no frame was received",
                    throttle_duration_sec=2.0,
                )
                if failures >= 30:
                    raise RuntimeError("Camera failed to provide 30 consecutive frames")
                time.sleep(0.1)
                continue
            failures = 0
            message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = frame_id
            if not rclpy.ok():
                break
            publisher.publish(message)
            node.get_logger().info(
                "Publishing images: %d x %d" % (message.width, message.height),
                throttle_duration_sec=5.0,
            )
            time.sleep(max(0.0, 1.0 / fps - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    except Exception as error:
        # Context shutdown can interrupt spin or publish during Ctrl+C.
        if rclpy.ok():
            if node is not None:
                node.get_logger().fatal(f"Camera publisher stopped: {error}")
            else:
                print(f"Camera publisher stopped: {error}")
            exit_code = 1
    finally:
        if capture is not None:
            capture.release()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
