import math
import os
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Keyboard:
    def __enter__(self):
        if os.name == "nt":
            import msvcrt
            self.console = msvcrt
        else:
            import select
            import termios
            import tty
            if not sys.stdin.isatty():
                raise RuntimeError("Run in an interactive terminal.")
            self.select = select
            self.termios = termios
            self.settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def read(self):
        if os.name == "nt":
            return self.console.getwch().lower() if self.console.kbhit() else ""
        if self.select.select([sys.stdin], [], [], 0)[0]:
            return os.read(sys.stdin.fileno(), 1).decode(errors="ignore").lower()
        return ""

    def __exit__(self, *args):
        if os.name != "nt":
            self.termios.tcsetattr(
                sys.stdin, self.termios.TCSADRAIN, self.settings
            )


class LidarStop:
    def __init__(self):
        self.lock = threading.Lock()
        self.distance = None
        self.received = None
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback, queue_size=1)

    def callback(self, scan):
        distances = []
        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= math.radians(20):
                if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                    distances.append(distance)
        with self.lock:
            self.distance = min(distances) if distances else None
            self.received = time.monotonic()

    def blocked(self):
        with self.lock:
            return (self.received is None or self.distance is None
                    or time.monotonic() - self.received > 1.0
                    or self.distance <= 0.30)


def main():
    rospy.init_node("part_b_lidar_stop")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    safety = LidarStop()
    linear = angular = 0.0
    print("W/S: linear +/- | A/D: turn +/- | Space: stop | Q: quit")
    print("Front obstacle or missing scan clears linear speed; press W/S again to move.")
    try:
        with Keyboard() as keyboard:
            while not rospy.is_shutdown():
                key = keyboard.read()
                if key in ("q", "\x03"):
                    break
                if key == "w":
                    linear += 0.02
                elif key == "s":
                    linear -= 0.02
                elif key == "a":
                    angular += 0.1
                elif key == "d":
                    angular -= 0.1
                elif key == " ":
                    linear = angular = 0.0
                linear = max(-0.2, min(0.2, linear))
                angular = max(-1.0, min(1.0, angular))
                if safety.blocked():
                    linear = 0.0  # Overrides both forward AND backward commands.
                command = Twist()
                command.linear.x = linear
                command.angular.z = angular
                pub.publish(command)
                time.sleep(0.1)
    finally:
        pub.publish(Twist())  # Best-effort stop on normal exit/Ctrl+C.


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
