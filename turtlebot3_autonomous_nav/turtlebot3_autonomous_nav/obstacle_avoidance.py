#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        
        self.safe_distance = self.get_parameter('safe_distance').get_parameter_value().double_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        
        self.get_logger().info('Obstacle Avoidance Node has been started')
    
    def scan_callback(self, msg):
        twist = Twist()
        
        ranges = msg.ranges
        num_readings = len(ranges)
        
        front_ranges = ranges[int(num_readings * 0.4):int(num_readings * 0.6)]
        left_ranges = ranges[int(num_readings * 0.6):int(num_readings * 0.8)]
        right_ranges = ranges[int(num_readings * 0.2):int(num_readings * 0.4)]
        
        front_min = min([r for r in front_ranges if msg.range_min < r < msg.range_max], default=msg.range_max)
        left_min = min([r for r in left_ranges if msg.range_min < r < msg.range_max], default=msg.range_max)
        right_min = min([r for r in right_ranges if msg.range_min < r < msg.range_max], default=msg.range_max)
        
        if front_min < self.safe_distance:
            self.get_logger().info(f'Obstacle detected at {front_min:.2f}m - Avoiding!')
            
            twist.linear.x = 0.0
            
            if left_min > right_min:
                twist.angular.z = self.angular_speed
                self.get_logger().info('Turning left')
            else:
                twist.angular.z = -self.angular_speed
                self.get_logger().info('Turning right')
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    
    try:
        rclpy.spin(obstacle_avoidance)
    except KeyboardInterrupt:
        pass
    
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
