#!/usr/bin/env python3
"""
SLAM Bringup Launch File for AVG Robot
========================================
Integrated launch file that brings up the complete SLAM system including:
    - Gazebo simulation with robot and world
    - Robot description and state publisher
    - Teleoperation (joystick control)
    - SLAM Toolbox for live mapping
    - RViz visualization with SLAM view

This is the complete pipeline for SLAM testing and mapping.

Author: AVG Team
ROS 2 Version: Jazzy
Gazebo Version: Harmonic

Usage:
    ros2 launch avg_bringup slam_bringup.launch.py
    
    # Optional: Disable RViz
    ros2 launch avg_bringup slam_bringup.launch.py use_rviz:=false
    
    # Optional: Use different world
    ros2 launch avg_bringup slam_bringup.launch.py world:=/path/to/world.sdf
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for complete SLAM system.
    
    Launch Arguments:
        - world: Path to Gazebo world file
        - use_sim_time: Use simulation time (default: true)
        - use_rviz: Launch RViz for SLAM visualization (default: true)
        - use_joystick: Enable joystick teleoperation (default: true)
        - x, y, z, yaw: Robot spawn position and orientation
    """
    
    # ========================================
    # Package Directories
    # ========================================
    
    pkg_avg_description = get_package_share_directory('avg_description')
    pkg_avg_teleop = get_package_share_directory('avg_teleop')
    pkg_avg_slam = get_package_share_directory('avg_slam')
    
    # ========================================
    # Default Paths
    # ========================================
    
    default_world = os.path.join(pkg_avg_description, 'worlds', 'warehouse.sdf')
    default_model = os.path.join(pkg_avg_description, 'urdf', 'robot.xacro')
    
    # ========================================
    # Launch Arguments
    # ========================================
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to Gazebo world file for simulation'
    )
    
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='Path to robot URDF/Xacro model file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for SLAM visualization'
    )
    
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='true',
        description='Enable joystick teleoperation control'
    )
    
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Robot spawn X coordinate'
    )
    
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Robot spawn Y coordinate'
    )
    
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.1',
        description='Robot spawn Z coordinate'
    )
    
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Robot spawn yaw orientation (radians)'
    )
    
    # ========================================
    # Launch Configurations
    # ========================================
    
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_joystick = LaunchConfiguration('use_joystick')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    
    # ========================================
    # 1. Simulation Launch (Gazebo + Robot)
    # ========================================
    
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_avg_description, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': world,
            'model': model,
            'use_sim_time': use_sim_time,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
            'rviz': 'false',  # We'll use SLAM-specific RViz config
            'gui': 'true',
        }.items()
    )
    
    # ========================================
    # 2. Teleoperation Launch
    # ========================================
    
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_avg_teleop, 'launch', 'teleop.launch.py')
        ),
        launch_arguments={
            'use_joystick': use_joystick,
            'cmd_vel_topic': '/cmd_vel',
        }.items()
    )
    
    # ========================================
    # 3. SLAM Launch (slam_toolbox + RViz)
    # ========================================
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_avg_slam, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
        }.items()
    )
    
    # ========================================
    # Launch Description Assembly
    # ========================================
    
    return LaunchDescription([
        # Launch arguments
        world_arg,
        model_arg,
        use_sim_time_arg,
        use_rviz_arg,
        use_joystick_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        
        # Launch files in order
        simulation_launch,
        teleop_launch,
        slam_launch,
    ])
