#!/usr/bin/env python3

import threading

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CameraViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None

        self.subscriber = rospy.Subscriber(
            "/evade/detection_image",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.frame = frame.copy()
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, str(error))

    def run(self):
        window = "TurtleBot Detection - Q to quit"
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        rospy.loginfo("Waiting for /evade/detection_image...")

        try:
            while not rospy.is_shutdown():
                with self.lock:
                    frame = self.frame

                if frame is not None:
                    cv2.imshow(window, frame)

                # Process window events on the main thread.
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("q"), 27):
                    break

                if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    break
        finally:
            self.subscriber.unregister()
            cv2.destroyAllWindows()
            rospy.signal_shutdown("Viewer closed")


if __name__ == "__main__":
    rospy.init_node("turtlebot_camera_viewer", anonymous=True)
    CameraViewer().run()
