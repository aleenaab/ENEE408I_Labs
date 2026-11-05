import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# TurtleBot3 Burger specifications
MAX_LINEAR = 0.22        # m/s
MAX_ANGULAR = 2.84       # rad/s
WHEEL_SEPARATION = 0.160  # meters

TEST_SECONDS = 2.0
STOP_SECONDS = 2.0
SPEED_LEVELS = [0.25, 0.50, 0.75, 1.00]


class DriveTest(Node):
    def __init__(self):
        super().__init__('drive_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tests = []

        motions = [
            'forward',
            'backward',
            'rotate clockwise',
            'rotate counterclockwise',
            'pivot: left wheel stopped',
            'pivot: right wheel stopped',
        ]

        for motion in motions:
            for fraction in SPEED_LEVELS:
                speed = MAX_LINEAR * fraction
                turn = MAX_ANGULAR * fraction

                if motion == 'forward':
                    linear, angular = speed, 0.0

                elif motion == 'backward':
                    linear, angular = -speed, 0.0

                elif motion == 'rotate clockwise':
                    linear, angular = 0.0, -turn

                elif motion == 'rotate counterclockwise':
                    linear, angular = 0.0, turn

                else:
                    # One wheel stays stationary while the other
                    # moves forward.
                    pivot_max = min(
                        MAX_LINEAR,
                        MAX_ANGULAR * WHEEL_SEPARATION,
                    )
                    moving_wheel_speed = pivot_max * fraction

                    linear = moving_wheel_speed / 2.0
                    angular = moving_wheel_speed / WHEEL_SEPARATION

                    if motion == 'pivot: right wheel stopped':
                        angular = -angular

                self.tests.append(
                    (motion, fraction, linear, angular)
                )

        self.index = 0
        self.moving = False
        self.done = False

        # Initial stop before starting the tests.
        self.deadline = self.now() + STOP_SECONDS
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

        self.get_logger().info(
            f'Starting {len(self.tests)} TurtleBot3 Burger tests '
            f'in {STOP_SECONDS:.0f} seconds.'
        )

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def tick(self):
        if self.done:
            return

        now = self.now()

        if now >= self.deadline:
            if self.moving:
                # Finish this movement and begin the stop period.
                self.moving = False
                self.index += 1
                self.deadline = now + STOP_SECONDS
                self.get_logger().info('STOP')

            elif self.index < len(self.tests):
                # Begin the next movement.
                self.moving = True
                self.deadline = now + TEST_SECONDS

                name, fraction, linear, angular = self.tests[self.index]

                self.get_logger().info(
                    f'{self.index + 1}/{len(self.tests)} | '
                    f'{name} | {fraction:.0%} | '
                    f'linear={linear:.3f} m/s | '
                    f'angular={angular:.3f} rad/s'
                )

            else:
                # All movements and the final stop are complete.
                self.pub.publish(Twist())
                self.done = True
                self.get_logger().info('All tests complete.')
                return

        # Default to zero velocity during stop periods.
        msg = Twist()

        if self.moving:
            _, _, linear, angular = self.tests[self.index]
            msg.linear.x = linear
            msg.angular.z = angular

        self.pub.publish(msg)

    def stop(self):
        self.timer.cancel()

        # Send repeated stop commands if ROS is still active.
        for _ in range(10):
            if not rclpy.ok():
                break
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    node = DriveTest()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
