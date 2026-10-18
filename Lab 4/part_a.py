import select
import sys
import termios
import tty
import rospy
from geometry_msgs.msg import Twist

def get_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1).lower()
    return ""

def move():
    rospy.init_node("keyboard_teleop", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10) 
    linear_speed = 0.0
    angular_speed = 0.0
    linear_step = 0.02 
    angular_step = 0.1  
    max_linear = 0.2
    max_angular = 1.0

    old_settings = termios.tcgetattr(sys.stdin)
    print(
        "\nKeyboard controls:\n"
        "  W / S : Increase/decrease linear velocity\n"
        "  A / D : Increase/decrease angular velocity\n"
        "  Space : Stop completely\n"
        "  Q     : Stop and quit\n"
    )

    try:
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