#!/usr/bin/env python3
"""
Launch file for complete teleoperation system for AVG robot.

Launches:
    - joy_node: ROS 2 joystick driver (reads from /dev/input/js0)
    - teleop_twist_joy: Converts joy messages to cmd_vel for joystick control

Usage:
    ros2 launch avg_teleop teleop.launch.py

Parameters:
    use_joystick: Enable joystick control (default: true)
    cmd_vel_topic: Output topic for velocity commands (default: /cmd_vel)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for complete teleoperation system."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('avg_teleop')
    params_dir = os.path.join(pkg_dir, 'params')
    teleop_params_file = os.path.join(params_dir, 'teleop.yaml')
    
    # Declare launch arguments
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='true',
        description='Enable joystick teleoperation'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic to publish velocity commands'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    # Joy node - ROS 2 joystick driver
    # Reads joystick input from device and publishes to /joy topic
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'dev': LaunchConfiguration('device'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(LaunchConfiguration('use_joystick')),
        output='screen'
    )
    
    # Teleop Twist Joy node - converts joy messages to cmd_vel
    # Subscribes to /joy and publishes to cmd_vel_topic
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[teleop_params_file],
        remappings=[
            ('/cmd_vel', LaunchConfiguration('cmd_vel_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('use_joystick')),
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_joystick_arg,
        cmd_vel_topic_arg,
        device_arg,
        
        # Nodes
        joy_node,
        teleop_twist_joy_node,
    ])
