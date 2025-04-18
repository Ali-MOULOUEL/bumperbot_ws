#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_ros import buffer, TransformListener, LookupException
from tf_transformations import euler_from_quaternion

class Pose:
    def __init__(self, px = 0, py = 0):
        self.px = px
        self.py = py

def coordinatesToPose(px, py, map_info: MapMetaData):
    pose = Pose()
    pose.x = (px - map_info.origin.position.x) / map_info.resolution
    pose.y = (py - map_info.origin.position.y) / map_info.resolution
    return pose

def poseOnMap(pose: Pose, map_info: MapMetaData):
    return pose.x < map_info.width and pose.x >=0 and pose.y < map_info.height and pose.y >= 0

def poseToCell(pose: Pose, map_info: MapMetaData):
    return pose.x + pose.y * map_info.width

class mapping_with_known_poses(Node):
    def __init__(self, name):
        super().__init__(self, name)

        self.declare_parameter('width', 50.0)   
        self.declare_parameter('height', 50.0) # Occupancy grid height
        self.declare_parameter('resolution', 0.1)

        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        resolution = self.get_parameter("resolution").value

        self.map_ = OccupancyGrid()
        self.map_.info.resolution = resolution
        self.map_.info.width = int(width / resolution) #  Number of grid cells
        self.map_.info.height = int(height / resolution)
        self.map_.info.origin.position.x = float(-round(width / 2.0))
        self.map_.info.origin.position.y = float(-round(height / 2.0))
        self.map_.header.frame_id = "odom"
        self.map_.data = [-1] * (self.map_.info.width * self.map_.info.height)

        self.map_pub = self.create_publisher(OccupancyGrid, "map", 1)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.tf_buffer = buffer.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def scan_callback(self, scan: LaserScan):
        try:
            t = self.tf_buffer.lookup_transform(self.map_.header.frame_id, scan.header.frame_id, rclpy.time.Time()) # Current transform between odom frame and laser frame
        except LookupException:
            self.get_logger().error("Unable to trnsform between /odom and /base_footprint")
            return
        
        (roll, pitch, yaw) = euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])

        robot_p = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, self.map_.info)
        if not poseOnMap(robot_p, self.map_.info):
            self.get_logger().error("Robot pose is outside the map")
            return
        
        for i in range(len(scan.ranges)):
            if math.isinf(scan.ranges[i]):
                continue
            angle = scan.angle_min + (i* scan.angle_increment) + yaw
            px = scan.ranges[i] * math.cos(angle)
            py = scan.ranges[i] * math.sin(angle)
            px += t.transform.translation.x
            py += t.transform.translation.y
            beam_p = coordinatesToPose(px, py, self.map_.info) # pose of each beam on map
            if not poseOnMap(beam_p, self.map_.info):
                self.get_logger().error("Beam pose is outside the map")
                continue
            cell = poseToCell(beam_p, self.map_.info)
            self.map_.data[cell] = 100 

    def timer_callback(self):
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_)

def main(args=None):
    rclpy.init(args=args)
    node = mapping_with_known_poses("mapping_with_known_poses")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()
# This code is a ROS2 node that creates an occupancy grid map based on laser scan data and known poses.
# It subscribes to laser scan data, transforms the data to the map frame, and updates the occupancy grid accordingly.
# The occupancy grid is published as a nav_msgs/OccupancyGrid message.