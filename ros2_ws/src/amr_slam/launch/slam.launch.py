#!/usr/bin/env python3
"""
SLAM Launch File for AMR Robot
================================
Launches slam_toolbox for online async mapping (Fresh Start).
This is the standard mode for creating a new map from scratch.

Usage Examples:
---------------
1. Start mapping (default params):
    ros2 launch amr_slam slam.launch.py

2. Start mapping with custom params:
    ros2 launch amr_slam slam.launch.py slam_params_file:=/path/to/my_params.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get Package directories
    pkg_amr_slam = get_package_share_directory('amr_slam')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # Default paths
    default_slam_params = os.path.join(pkg_amr_slam, 'params', 'slam_params.yaml')
    
    # ========================================
    # Launch Arguments
    # ========================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='Full path to the SLAM parameters YAML file'
    )
    
    # ========================================
    # Launch Configurations
    # ========================================
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # ========================================
    # SLAM Toolbox - Online Async Launch
    # ========================================
    
    # Include slam_toolbox's online_async_launch with custom parameters
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # ========================================
    # Launch Description Assembly
    # ========================================
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        slam_params_file_arg,
        
        # Nodes and includes
        slam_toolbox_launch,
    ])
