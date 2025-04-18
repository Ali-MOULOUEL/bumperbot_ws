#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped')
        self.subscription = self.create_subscription(
            Twist,
            '/key_vel',
            self.twist_callback,
            10)
        self.publisher = self.create_publisher(TwistStamped, '/input_joy/cmd_vel_stamped', 10)

    def twist_callback(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header = Header()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = msg
        self.publisher.publish(twist_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()