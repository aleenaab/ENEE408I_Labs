#!/usr/bin/env python3

import threading
import time

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


TURN_SPEED = 0.4       # radians/second
MIN_RED_AREA = 300
FRAME_TIMEOUT = 0.5    # seconds


class RedTurn:
    def __init__(self):
        self.bridge = CvBridge()
        self.kernel = np.ones((5, 5), np.uint8)
        self.lock = threading.Lock()

        self.turn_direction = 0.0
        self.last_frame_time = None
        self.last_status = None
        self.stopping = False

        self.cmd_pub = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=1
        )
        self.image_pub = rospy.Publisher(
            "/evade/detection_image", Image, queue_size=1
        )

        self.image_sub = rospy.Subscriber(
            "/camera/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )

        self.timer = rospy.Timer(
            rospy.Duration(0.05), self.publish_command
        )
        rospy.on_shutdown(self.stop)

        rospy.loginfo("Ready: red detection with rotation only.")

    def image_callback(self, msg):
        received_at = time.monotonic()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
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
            rospy.logerr_throttle(2.0, str(error))

    def publish_command(self, event):
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
            rospy.loginfo(status)
            self.last_status = status

    def stop(self):
        with self.lock:
            self.stopping = True
            self.turn_direction = 0.0

        self.timer.shutdown()

        # Best-effort stop commands during shutdown.
        for _ in range(5):
            try:
                self.cmd_pub.publish(Twist())
            except rospy.ROSException:
                break
            time.sleep(0.05)


if __name__ == "__main__":
    rospy.init_node("red_turn")
    node = RedTurn()
    rospy.spin()
