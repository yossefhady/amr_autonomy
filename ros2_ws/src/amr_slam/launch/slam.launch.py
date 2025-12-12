#!/usr/bin/env python3
"""
SLAM Launch File for AMR Robot
================================
Launches slam_toolbox for online async mapping with RViz visualization.

"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate launch description for SLAM with slam_toolbox.
    
    Launch Arguments:
        - use_sim_time: Use simulation time (default: true)
        - slam_params_file: Path to slam_toolbox params (default: slam_params.yaml)
        - use_rviz: Launch RViz for visualization (default: true)
        - rviz_config: RViz config file path (default: slam_view.rviz)
    """
    
    # Package directories
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
