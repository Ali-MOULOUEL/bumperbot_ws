import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QosDurabilityPolicy
from std_msgs.msg import String


class SimpleQosSubscriber(Node):

    def __init__(self):
        super().__init__("simple_subscriber")

        self.qos_profile_sub = QoSProfile(depth=10)

        self.declare_parameter("reliability", "system_default")
        self.declare_parameter("durability", "system_default")

        reliability = self.get_parameter("reliability").get_parameter_value().string_value
        durbaility = self.get_parameter("durability").get_parameter_value().string_value

        if reliability == "best_effort":
            self.qos_profile_sub.reliability = QoSReliabilityPolicy.BEST_EFFORT
            self.get_logger().info("Reliability: BEST_EFFORT")
        elif reliability == "reliable":
            self.qos_profile_sub.reliability = QoSReliabilityPolicy.RELIABLE
            self.get_logger().info("Reliability: RELIABLE")
        elif reliability == "system_default":
            self.qos_profile_sub.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info("Reliability: SYSTEM_DEFAULT")
        else:
            self.get_logger().error("Invalid reliability parameter: %s" % reliability)
            return

        if durbaility == "volatile":
            self.qos_profile_sub.durability = QosDurabilityPolicy.VOLATILE
            self.get_logger().info("Durability: VOLATILE")
        elif durbaility == "transient_local":
            self.qos_profile_sub.durability = QosDurabilityPolicy.TRANSIENT_LOCAL
            self.get_logger().info("Durability: TRANSIENT_LOCAL")
        elif durbaility == "system_default":
            self.qos_profile_sub.durability = QosDurabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info("Durability: SYSTEM_DEFAULT")
        else:
            self.get_logger().error("Invalid durability parameter: %s" % durbaility)
            return

        self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, self.qos_profile_sub)

    def msgCallback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)


def main():
    rclpy.init()

    simple_qos_publisher = SimpleQosSubscriber()
    rclpy.spin(simple_qos_publisher)
    
    simple_qos_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()