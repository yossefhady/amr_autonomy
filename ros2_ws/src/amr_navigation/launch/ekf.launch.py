#!/usr/bin/env python3
"""
EKF Localization Launch File
============================
Launches the robot_localization Extended Kalman Filter (EKF) node.
This node fuses data from:
1. Wheel Odometry (/odom)
2. IMU Data (/imu)

It publishes:
- /odometry/filtered (Fused Odometry)
- TF: odom -> base_footprint

Usage:
------
ros2 launch amr_navigation ekf.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    
    # Default paths
    ekf_config_path = os.path.join(pkg_amr_navigation, 'params', 'ekf.yaml')

    # ========================================
    # Launch Arguments
    # ========================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ========================================
    # EKF Node Launch
    # ========================================
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path, 
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('odometry/filtered', '/odometry/filtered')]
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_node
    ])
