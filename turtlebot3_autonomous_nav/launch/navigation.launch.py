import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    pkg_share = get_package_share_directory('turtlebot3_autonomous_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    rviz_config_file = os.path.join(pkg_share, 'config', 'navigation.rviz')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'slam': 'True'
        }.items()
    )
    
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
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
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(sensor_subscriber_cmd)
    
    return ld
