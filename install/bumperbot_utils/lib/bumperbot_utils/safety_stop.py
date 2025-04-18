#!/usr/bin/env python3
import math
import rclpy
from enum import Enum
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from twist_mux_msgs.action import JoyTurbo
from visualization_msgs.msg import Marker, MarkerArray


class State(Enum):
    FREE = 0  # Robot is free to move
    WARNING = 1  # Robot is in a warning state
    DANGER = 2  # Robot is in a danger state


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop_node')
        self.declare_parameter('danger_distance', 0.2)
        self.declare_parameter('warning_distance', 0.6)
        self.declare_parameter('scan_topic', "scan")
        self.declare_parameter("safety_stop_topic", "safety_stop")
        self.danger_distance = self.get_parameter(
            "danger_distance").get_parameter_value().double_value
        self.warning_distance = self.get_parameter(
            "warning_distance").get_parameter_value().double_value
        self.scan_topic = self.get_parameter(
            "scan_topic").get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter(
            "safety_stop_topic").get_parameter_value().string_value

        self.state = State.FREE
        self.previous_state = State.FREE
        self.is_first_msg = True

        self.laser_sub = self.create_subscription(LaserScan, self.scan_topic, self.laser_callback, 10)
        self.safety_stop_pub = self.create_publisher(Bool, self.safety_stop_topic, 10)
        self.zones_pub = self.create_publisher(MarkerArray, "zones", 10)

        self.decrease_speed_client = ActionClient(self, JoyTurbo, "joy_turbo_decrease")
        self.increase_speed_client = ActionClient(self, JoyTurbo, "joy_turbo_increase")

        while not self.decrease_speed_client.wait_for_server(timeout_sec=1) and rclpy.ok():
            self.get_logger().warn("Action /joy_turbo_decrease not available, waiting again...")

        while not self.increase_speed_client.wait_for_server(timeout_sec=1) and rclpy.ok():
            self.get_logger().warn("Action /joy_turbo_increase not available, waiting again...")

        self.zones = MarkerArray()
        warning_zone = Marker()
        warning_zone.action = Marker.ADD
        warning_zone.type = Marker.CYLINDER
        warning_zone.scale.z = 0.001
        warning_zone.scale.x = 2 * self.warning_distance
        warning_zone.scale.y = 2 * self.warning_distance
        warning_zone.color.r = 1.0
        warning_zone.color.g = 0.984
        warning_zone.color.b = 0.0
        warning_zone.color.a = 0.5

        danger_zone = Marker()
        danger_zone.id = 1
        danger_zone.action = Marker.ADD
        danger_zone.type = Marker.CYLINDER
        danger_zone.scale.z = 0.001
        danger_zone.scale.x = 2 * self.danger_distance
        danger_zone.scale.y = 2 * self.danger_distance
        danger_zone.color.r = 1.0
        danger_zone.color.g = 0.0
        danger_zone.color.b = 0.0
        danger_zone.pose.position.z = 0.01 #Pour eviter que les deux marker se superposent
        self.zones.markers = [warning_zone, danger_zone]

    def laser_callback(self, msg):
        self.state = State.FREE
        for range_value in msg.ranges:
            if not math.isinf(range_value) and range_value <= self.warning_distance:
                self.state = State.WARNING
                if range_value <= self.danger_distance:
                    self.state = State.DANGER
                    break

        if self.state != self.previous_state:
            is_safety_stop = Bool()
            if self.state == State.WARNING:
                is_safety_stop.data = False
                self.zones.markers[0].color.a = 1.0
                self.zones.markers[1].color.a = 0.5
                self.decrease_speed_client.send_goal_async(JoyTurbo.Goal())
            elif self.state == State.DANGER:
                is_safety_stop.data = True
                self.zones.markers[0].color.a = 1.0
                self.zones.markers[1].color.a = 1.0
            elif self.state == State.FREE:
                is_safety_stop.data = False
                self.zones.markers[0].color.a = 0.5
                self.zones.markers[1].color.a = 0.5
                self.increase_speed_client.send_goal_async(JoyTurbo.Goal())

            self.previous_state = self.state
            self.safety_stop_pub.publish(is_safety_stop)
        
        if self.is_first_msg:
            for zone in self.zones.markers:
                zone.header.frame_id = msg.header.frame_id
            
            self.is_first_msg = False
        
        self.zones_pub.publish(self.zones)



def main():
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
