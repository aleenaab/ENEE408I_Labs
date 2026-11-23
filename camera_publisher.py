#!/usr/bin/env python3
"""Publish a locally accessible OpenCV camera as ROS 1 sensor_msgs/Image."""
import math
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def main():
    rospy.init_node("camera_publisher")
    device = rospy.get_param("~device", 0)
    if isinstance(device, str) and device.isdigit():
        device = int(device)
    topic = rospy.get_param("~image_topic", "/camera/image_raw")
    frame_id = rospy.get_param("~frame_id", "camera_optical_frame")
    fps = float(rospy.get_param("~fps", 15.0))
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be a positive finite number")

    capture = cv2.VideoCapture(device)
    try:
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
        publisher = rospy.Publisher(topic, Image, queue_size=1)
        rospy.loginfo("Camera opened; publishing on %s. Ctrl+C to exit.", topic)
        failures = 0
        while not rospy.is_shutdown():
            started = time.monotonic()
            ok, frame = capture.read()
            if not ok or frame is None:
                failures += 1
                rospy.logwarn_throttle(2.0, "Camera opened but no frame was received")
                if failures >= 30:
                    raise RuntimeError("Camera failed to provide 30 consecutive frames")
                time.sleep(0.1)
                continue
            failures = 0
            message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = frame_id
            publisher.publish(message)
            rospy.loginfo_throttle(
                5.0, "Publishing images: %d x %d" % (message.width, message.height)
            )
            time.sleep(max(0.0, 1.0 / fps - (time.monotonic() - started)))
    finally:
        capture.release()


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    except Exception as error:
        rospy.logfatal("Camera publisher stopped: %s", error)
        raise SystemExit(1)
