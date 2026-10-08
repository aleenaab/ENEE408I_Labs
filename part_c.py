import math
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan


class VisionControl:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.distance = self.scan_time = None
        self.error = self.image_time = None
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        topic = rospy.get_param("~camera_topic", "/camera/image_raw")
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        self.image_sub = rospy.Subscriber(
            topic, Image, self.image_callback, queue_size=1, buff_size=2**24
        )

    def scan_callback(self, scan):
        distances = []
        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= math.radians(20):
                if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                    distances.append(distance)
        with self.lock:
            self.distance = min(distances) if distances else None
            self.scan_time = time.monotonic()

    def image_callback(self, message):
        received = time.monotonic()
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, str(error))
            with self.lock:
                self.image_time = None
                self.error = None
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Red occupies both ends of OpenCV's hue scale.
        low = cv2.inRange(hsv, (0, 100, 70), (10, 255, 255))
        high = cv2.inRange(hsv, (170, 100, 70), (179, 255, 255))
        mask = cv2.bitwise_or(low, high)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        error = None
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) >= 300:
                moments = cv2.moments(largest)
                if moments["m00"] > 0:
                    cx = moments["m10"] / moments["m00"]
                    half_width = frame.shape[1] / 2.0
                    error = (cx - half_width) / half_width
        with self.lock:
            self.error = error
            self.image_time = received

    def run(self):
        try:
            while not rospy.is_shutdown():
                now = time.monotonic()
                with self.lock:
                    distance, scan_time = self.distance, self.scan_time
                    error, image_time = self.error, self.image_time
                command = Twist()
                ready = (scan_time is not None and image_time is not None
                         and now - scan_time < 1.0 and now - image_time < 1.0
                         and distance is not None)
                if ready:
                    if error is None:
                        command.angular.z = 0.2  # Search in place.
                    else:
                        command.linear.x = 0.05
                        command.angular.z = max(-0.5, min(0.5, -0.6 * error))
                    if distance <= 0.30:
                        command.linear.x = 0.0
                # Missing/stale sensors cause a complete stop.
                self.pub.publish(command)
                time.sleep(0.1)
        finally:
            self.pub.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("part_c_vision_control")
    try:
        VisionControl().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
