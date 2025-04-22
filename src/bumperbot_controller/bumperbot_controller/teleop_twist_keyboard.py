#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, '/key_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.linear_speed = 0.5
        self.angular_speed = 0.5
        self.get_logger().info("Use arrow keys to move the robot. Press 'q' to quit.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(3)  # Read 3 characters for arrow keys
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                twist = Twist()

                if key == '\x1b[A':  # Up arrow key
                    twist.linear.x = self.linear_speed
                elif key == '\x1b[B':  # Down arrow key
                    twist.linear.x = -self.linear_speed
                elif key == '\x1b[D':  # Left arrow key
                    twist.angular.z = self.angular_speed
                elif key == '\x1b[C':  # Right arrow key
                    twist.angular.z = -self.angular_speed
                elif key == 'q':  # Quit
                    break
                else:  # Stop
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.publisher.publish(twist)
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()