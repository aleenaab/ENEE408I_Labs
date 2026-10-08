#!/usr/bin/env python3

import math
import select
import sys
import termios
import threading
import time
import tty

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class LidarSafety:
    def __init__(self):
        self.lock = threading.Lock()
        self.distance = None
        self.last_scan = None

        self.subscriber = rospy.Subscriber(
            "/scan",
            LaserScan,
            self.scan_callback,
            queue_size=1,
        )

    def scan_callback(self, scan):
        front_distances = []

        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            # Check 20 degrees to either side of straight ahead.
            if abs(angle) <= math.radians(20):
                if (
                    math.isfinite(distance)
                    and scan.range_min <= distance <= scan.range_max
                ):
                    front_distances.append(distance)

        with self.lock:
            self.distance = (
                min(front_distances) if front_distances else None
            )
            self.last_scan = time.monotonic()

    def status(self):
        with self.lock:
            distance = self.distance
            last_scan = self.last_scan

        if last_scan is None:
            return True, "BLOCKED: waiting for /scan"

        if time.monotonic() - last_scan > 1.0:
            return True, "BLOCKED: LiDAR data is outdated"

        if distance is None:
            return True, "BLOCKED: no valid front LiDAR readings"

        if distance <= 0.30:
            return True, f"BLOCKED: obstacle at {distance:.2f} m"

        return False, f"CLEAR: front distance {distance:.2f} m"


def get_key():
    # Same keyboard-reading approach as your working Part A.
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1).lower()
    return ""


def move():
    if not sys.stdin.isatty():
        raise RuntimeError("Run this script in an interactive Ubuntu terminal.")

    rospy.init_node("keyboard_lidar_teleop", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    safety = LidarSafety()

    linear_speed = 0.0
    angular_speed = 0.0
    last_key = "-"
    last_display = 0.0

    old_settings = termios.tcgetattr(sys.stdin)

    print(
        "\nPart B: Keyboard control with LiDAR stopping\n"
        "W / S : Increase / decrease linear velocity\n"
        "A / D : Increase / decrease turning velocity\n"
        "Space : Stop completely\n"
        "Q     : Stop and quit\n\n"
        "Click inside this terminal before pressing keys.\n"
        "Releasing a key does not stop the robot.\n"
        "After a LiDAR stop, press W/S again when clear.\n"
    )

    try:
        tty.setcbreak(sys.stdin.fileno())

        while not rospy.is_shutdown():
            key = get_key()

            if key:
                last_key = "SPACE" if key == " " else repr(key)

            if key in ("q", "\x03"):
                break
            elif key == "w":
                linear_speed += 0.02
            elif key == "s":
                linear_speed -= 0.02
            elif key == "a":
                angular_speed += 0.1
            elif key == "d":
                angular_speed -= 0.1
            elif key == " ":
                linear_speed = 0.0
                angular_speed = 0.0

            linear_speed = max(-0.2, min(0.2, linear_speed))
            angular_speed = max(-1.0, min(1.0, angular_speed))

            blocked, status_text = safety.status()

            if blocked:
                # Block forward AND backward movement.
                # Turning remains available.
                linear_speed = 0.0

            command = Twist()
            command.linear.x = linear_speed
            command.angular.z = angular_speed
            pub.publish(command)

            now = time.monotonic()
            if key or now - last_display >= 0.5:
                text = (
                    f"Key: {last_key} | "
                    f"Linear: {linear_speed:+.2f} m/s | "
                    f"Turn: {angular_speed:+.2f} rad/s | "
                    f"{status_text}"
                )
                print("\r" + text.ljust(130), end="", flush=True)
                last_display = now

            time.sleep(0.1)  # Publish at approximately 10 Hz.

    finally:
        try:
            pub.publish(Twist())  # Attempt to stop on exit.
        finally:
            termios.tcsetattr(
                sys.stdin, termios.TCSADRAIN, old_settings
            )
            print("\nTeleoperation ended.")


if __name__ == "__main__":
    try:
        move()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass