#!/usr/bin/env python3
"""
Nav2 Launch File for AMR Robot
================================
Launches Nav2 navigation stack with RViz visualization.

Author: AMR Team
ROS 2 Version: Jazzy
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate launch description for Nav2 navigation stack.
    
    Launch Arguments:
        - use_sim_time: Use simulation time (default: true)
        - map: Path to map YAML file (default: my_warehouse_map.yaml)
        - params_file: Path to Nav2 params (default: nav2_params.yaml)
        - use_rviz: Launch RViz for visualization (default: true)
        - rviz_config: RViz config file path (default: nav2_view.rviz)
    """
    
    # Package directories
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Default paths
    default_map_file = os.path.join(pkg_amr_navigation, 'maps', 'my_warehouse_map.yaml')
    default_params_file = os.path.join(pkg_amr_navigation, 'params', 'nav2_params.yaml')
    default_rviz_config = os.path.join(pkg_amr_navigation, 'rviz', 'nav2_view.rviz')
    
    # ========================================
    # Launch Arguments
    # ========================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to the map YAML file'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the Nav2 parameters YAML file'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz for visualization'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to RViz config file'
    )
    
    # ========================================
    # Launch Configurations
    # ========================================
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # ========================================
    # Nav2 Bringup Launch
    # ========================================
    
    # Include nav2_bringup's bringup_launch.py
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # ========================================
    # RViz2 Visualization
    # ========================================
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        respawn=False
    )
    
    # ========================================
    # Launch Description Assembly
    # ========================================
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        map_arg,
        params_file_arg,
        use_rviz_arg,
        rviz_config_arg,
        
        # Nodes and includes
        nav2_bringup_launch,
        rviz_node,
    ])
