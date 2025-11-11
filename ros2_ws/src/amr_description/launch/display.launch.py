#!/usr/bin/env python3
"""
Launch file for displaying the warehouse AGV robot in RViz.
This file launches:
  - robot_state_publisher: publishes the robot's TF transforms
  - joint_state_publisher_gui: allows manual control of joint positions
  - rviz2: visualization tool with pre-configured settings
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for displaying the robot."""
    
    # Get the package directory
    pkg_description = get_package_share_directory('amr_description')
    
    # Paths to files
    default_model_path = os.path.join(pkg_description, 'urdf', 'robot.xacro')
    default_rviz_config_path = os.path.join(pkg_description, 'rviz', 'display_config.rviz')
    
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot URDF/Xacro file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    use_gui_arg = DeclareLaunchArgument(
        name='use_gui',
        default_value='true',
        description='Launch joint_state_publisher_gui if true'
    )
    
    # Get launch configurations
    model = LaunchConfiguration('model')
    rvizconfig = LaunchConfiguration('rvizconfig')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')
    
    # Process the URDF/Xacro file
    robot_description = ParameterValue(
        Command(['xacro ', model]),
        value_type=str
    )
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint State Publisher GUI Node (for manual joint control)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_gui)
    )
    
    # Joint State Publisher Node (non-GUI fallback)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_gui)
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create and return launch description
    return LaunchDescription([
        model_arg,
        rviz_arg,
        use_sim_time_arg,
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node
    ])
