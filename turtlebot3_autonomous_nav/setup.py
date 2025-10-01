from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_autonomous_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Autonomous navigation package for TurtleBot3 using ROS2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_publisher = turtlebot3_autonomous_nav.velocity_publisher:main',
            'sensor_subscriber = turtlebot3_autonomous_nav.sensor_subscriber:main',
            'obstacle_avoidance = turtlebot3_autonomous_nav.obstacle_avoidance:main',
        ],
    },
)
