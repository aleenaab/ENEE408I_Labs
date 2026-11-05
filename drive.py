import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class BasicDrive(Node):
    def __init__(self, speed=0.1, seconds=3.0):
        super().__init__('basic_drive')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = speed
        self.end = self.get_clock().now().nanoseconds * 1e-9 + seconds
        self.create_timer(0.05, self.tick)  # 20 Hz

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        msg = Twist()
        if now < self.end:
            msg.linear.x = self.speed      # m/s, max 0.22
            msg.angular.z = 0.0            # rad/s, positive turns left
        self.pub.publish(msg)              # zeros after the time is up
        if now > self.end + 0.5:
            raise SystemExit


def main():
    rclpy.init()
    node = BasicDrive(speed=0.1, seconds=3.0)  # about 0.3 m
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    node.pub.publish(Twist())  # make sure it stops
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
