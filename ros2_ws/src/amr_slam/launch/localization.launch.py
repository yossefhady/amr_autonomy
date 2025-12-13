#!/usr/bin/env python3
"""
Localization Launch File for AMR Robot
======================================
Launches slam_toolbox in localization mode (alternative approach for AMCL).
It loads a serialized pose graph and localizes the robot within it
without updating the map.

Usage Examples:
---------------
1. Localize in 'medium_warehouse':
    ros2 launch amr_slam localization.launch.py world_name:=medium_warehouse

2. Use specific params file:
    ros2 launch amr_slam localization.launch.py slam_params_file:=/path/to/params.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_amr_slam = get_package_share_directory('amr_slam')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # Default paths
    default_slam_params = os.path.join(pkg_amr_slam, 'params', 'localization_params.yaml')

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

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value='medium_warehouse',
        description='Name of the world (e.g. medium_warehouse)'
    )
    
    # Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    world = LaunchConfiguration('world')

    # Construct map path
    map_file_path = PathJoinSubstitution([
        pkg_amr_slam, 'maps', world, world
    ])
    
    # ========================================
    # SLAM Toolbox (Standard Launch + Param Injection)
    # ========================================

    slam_launch = GroupAction([
        # Inject the map filename
        SetParameter(name='map_file_name', value=map_file_path),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam_toolbox, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': use_sim_time,
            }.items()
        )
    ])
    
    return LaunchDescription([
        use_sim_time_arg,
        slam_params_file_arg,
        world_arg,
        slam_launch,
    ])
