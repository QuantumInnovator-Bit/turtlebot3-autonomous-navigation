#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        self.current_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.min_distance = float('inf')
        self.min_angle = 0.0
        
        self.get_logger().info('Sensor Subscriber Node has been started')
    
    def odom_callback(self, msg):
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_position['theta'] = math.atan2(siny_cosp, cosy_cosp)
        
        self.get_logger().info(f'Position: x={self.current_position["x"]:.2f}, y={self.current_position["y"]:.2f}, theta={self.current_position["theta"]:.2f}', throttle_duration_sec=1.0)
    
    def scan_callback(self, msg):
        ranges = msg.ranges
        
        valid_ranges = [r for r in ranges if msg.range_min < r < msg.range_max]
        
        if valid_ranges:
            self.min_distance = min(valid_ranges)
            min_index = ranges.index(self.min_distance)
            self.min_angle = msg.angle_min + min_index * msg.angle_increment
            
            self.get_logger().info(f'Min obstacle distance: {self.min_distance:.2f}m at angle: {math.degrees(self.min_angle):.2f}°', throttle_duration_sec=1.0)
        else:
            self.min_distance = float('inf')
            self.min_angle = 0.0
    
    def get_position(self):
        return self.current_position
    
    def get_min_obstacle_distance(self):
        return self.min_distance, self.min_angle

def main(args=None):
    rclpy.init(args=args)
    sensor_subscriber = SensorSubscriber()
    
    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        pass
    
    sensor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
