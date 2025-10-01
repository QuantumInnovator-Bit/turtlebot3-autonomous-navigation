import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    obstacle_avoidance_cmd = Node(
        package='turtlebot3_autonomous_nav',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    sensor_subscriber_cmd = Node(
        package='turtlebot3_autonomous_nav',
        executable='sensor_subscriber',
        name='sensor_subscriber',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(obstacle_avoidance_cmd)
    ld.add_action(sensor_subscriber_cmd)
    
    return ld
