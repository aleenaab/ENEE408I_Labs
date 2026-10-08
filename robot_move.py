#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


def get_key():
    """Return a key if available, without waiting for input."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1).lower()
    return ""


def move():
    rospy.init_node("keyboard_teleop", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)  # Publish at 10 Hz

    linear_speed = 0.0
    angular_speed = 0.0

    linear_step = 0.02   # m/s per key press
    angular_step = 0.1   # rad/s per key press

    # Adjust these limits to match your robot.
    max_linear = 0.2
    max_angular = 1.0

    if not sys.stdin.isatty():
        raise RuntimeError("Run this program in an interactive terminal.")

    old_settings = termios.tcgetattr(sys.stdin)

    print(
        "\nKeyboard controls:\n"
        "  W / S : Increase / decrease linear velocity\n"
        "  A / D : Increase / decrease angular velocity\n"
        "  Space : Stop completely\n"
        "  Q     : Stop and quit\n"
        "\nVelocity persists until changed or stopped.\n"
    )

    try:
        # Read individual keys without pressing Enter.
        tty.setcbreak(sys.stdin.fileno())

        while not rospy.is_shutdown():
            key = get_key()

            if key == "w":
                linear_speed += linear_step
            elif key == "s":
                linear_speed -= linear_step
            elif key == "a":
                angular_speed += angular_step
            elif key == "d":
                angular_speed -= angular_step
            elif key == " ":
                linear_speed = 0.0
                angular_speed = 0.0
            elif key == "q":
                break

            # Keep velocities within the configured limits.
            linear_speed = max(
                -max_linear, min(max_linear, linear_speed)
            )
            angular_speed = max(
                -max_angular, min(max_angular, angular_speed)
            )

            vel_msg = Twist()
            vel_msg.linear.x = linear_speed
            vel_msg.angular.z = angular_speed
            pub.publish(vel_msg)

            if key:
                print(
                    f"\rLinear: {linear_speed:+.2f} m/s | "
                    f"Angular: {angular_speed:+.2f} rad/s    ",
                    end="",
                    flush=True,
                )

            rate.sleep()

    finally:
        # Attempt to stop on exit, and always restore the terminal.
        try:
            pub.publish(Twist())
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