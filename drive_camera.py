#!/usr/bin/env python3

import signal
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


TURN_SPEED = 0.4       # radians/second
MIN_RED_AREA = 300
FRAME_TIMEOUT = 0.5    # seconds


class RedTurn(Node):
    def __init__(self):
        super().__init__("red_turn")
        self.bridge = CvBridge()
        self.kernel = np.ones((5, 5), np.uint8)
        self.lock = threading.Lock()

        self.turn_direction = 0.0
        self.last_frame_time = None
        self.last_status = None
        self.stopping = False

        # Separate groups keep the watchdog running during image processing.
        self.image_group = MutuallyExclusiveCallbackGroup()
        self.command_group = MutuallyExclusiveCallbackGroup()
        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.image_pub = self.create_publisher(
            Image, "/evade/detection_image", 1
        )
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, camera_qos,
            callback_group=self.image_group,
        )
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            0.05, self.publish_command,
            callback_group=self.command_group, clock=self.watchdog_clock,
        )
        self.get_logger().info("Ready: red detection with rotation only.")

    def image_callback(self, msg):
        received_at = time.monotonic()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
            width = frame.shape[1]

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(
                hsv, (0, 100, 70), (10, 255, 255)
            )
            mask2 = cv2.inRange(
                hsv, (170, 100, 70), (179, 255, 255)
            )
            mask = cv2.bitwise_or(mask1, mask2)

            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, self.kernel
            )
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE, self.kernel
            )

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            target = (
                max(contours, key=cv2.contourArea)
                if contours else None
            )
            detected = (
                target is not None
                and cv2.contourArea(target) > MIN_RED_AREA
            )

            with self.lock:
                if self.stopping:
                    return

                if detected:
                    x, y, w, h = cv2.boundingRect(target)
                    cx = x + w / 2.0

                    # Keep the chosen direction until detection is lost.
                    if self.turn_direction == 0.0:
                        self.turn_direction = (
                            -1.0 if cx <= width / 2.0 else 1.0
                        )

                    direction = (
                        "LEFT" if self.turn_direction > 0 else "RIGHT"
                    )
                    status = f"Red detected: turning {direction}"
                else:
                    self.turn_direction = 0.0
                    status = "STOP: no red detected"

                self.last_frame_time = received_at
                self.log_status(status)

            if detected:
                cv2.rectangle(
                    frame, (x, y), (x + w, y + h),
                    (0, 255, 0), 2
                )

            cv2.putText(
                frame, status, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2
            )

            output = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            output.header = msg.header
            self.image_pub.publish(output)

        except (CvBridgeError, cv2.error) as error:
            with self.lock:
                self.turn_direction = 0.0
                self.last_frame_time = None
            self.get_logger().error(str(error), throttle_duration_sec=2.0)

    def publish_command(self):
        with self.lock:
            if self.stopping:
                return

            fresh = (
                self.last_frame_time is not None
                and time.monotonic() - self.last_frame_time
                < FRAME_TIMEOUT
            )

            command = Twist()  # All linear velocities remain zero.

            if fresh:
                command.angular.z = (
                    self.turn_direction * TURN_SPEED
                )
            else:
                self.turn_direction = 0.0
                self.log_status("STOP: no recent camera frames")

            self.cmd_pub.publish(command)

    def log_status(self, status):
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def stop(self):
        with self.lock:
            self.stopping = True
            self.turn_direction = 0.0

        self.timer.cancel()

        # Best-effort stop commands during shutdown.
        for _ in range(5):
            try:
                self.cmd_pub.publish(Twist())
            except Exception:  # Shutdown publishing is best effort.
                break
            time.sleep(0.05)


def main(args=None):
    # Keep the context alive until stop commands have been published.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    shutdown_requested = threading.Event()
    previous_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(
            signum, lambda *_: shutdown_requested.set()
        )

    node = None
    executor = None
    try:
        node = RedTurn()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        while rclpy.ok() and not shutdown_requested.is_set():
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    main()
