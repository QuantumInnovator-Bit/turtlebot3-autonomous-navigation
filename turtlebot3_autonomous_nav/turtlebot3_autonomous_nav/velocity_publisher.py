#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publish_velocity)
        
        self.declare_parameter('linear_velocity', 0.2)
        self.declare_parameter('angular_velocity', 0.5)
        
        self.get_logger().info('Velocity Publisher Node has been started')
        
    def publish_velocity(self):
        msg = Twist()
        
        linear_vel = self.get_parameter('linear_velocity').get_parameter_value().double_value
        angular_vel = self.get_parameter('angular_velocity').get_parameter_value().double_value
        
        msg.linear.x = linear_vel
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_vel
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    
    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    
    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
