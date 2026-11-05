import math
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

class ObjectDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.ball_diameter = 0.22 #need to change based on the ball
        self.focal_length = 554.0
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw", Image, self.image_callback,
            queue_size=1, buff_size=2**24
        )
        self.position_pub = rospy.Publisher(
            "/evade/detection", PointStamped, queue_size=1
        )
        self.image_pub = rospy.Publisher(
            "/evade/detection_image", Image, queue_size=1
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, (0, 100, 70), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 100, 70), (179, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 300:
                x, y, w, h = cv2.boundingRect(largest)
                cx = x + w // 2
                cy = y + h // 2
                distance = (
                    self.focal_length * self.ball_diameter / max(w, h)
                )
                x_camera = (cx - width / 2) * distance / self.focal_length
                y_camera = (cy - height / 2) * distance / self.focal_length
                point = PointStamped()
                point.header = msg.header
                point.point.x = x_camera
                point.point.y = y_camera
                point.point.z = distance
                self.position_pub.publish(point)
                cv2.rectangle(
                    frame, (x, y), (x + w, y + h), (0, 255, 0), 2
                )
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

                cv2.putText(
                    frame, f"Red Ball: {distance:.2f} m",
                    (x, max(y - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )
        output = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        output.header = msg.header
        self.image_pub.publish(output)

if __name__ == "__main__":
    rospy.init_node("object_detector")
    ObjectDetector()
    rospy.spin()