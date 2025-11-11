#!/usr/bin/env python3
"""
Navigation Bringup Launch File for AMR Robot
=============================================
Integrated launch file that brings up the complete navigation system including:
    - Gazebo simulation with robot and world
    - Robot description and state publisher
    - Teleoperation (joystick control)
    - Nav2 navigation stack with localization
    - RViz visualization with navigation view

This is the complete pipeline for navigation testing and autonomous operation.

Author: AMR Team
ROS 2 Version: Jazzy
Gazebo Version: Harmonic

Usage:
    ros2 launch amr_bringup bringup_sim.launch.py
    
    # Optional: Disable RViz
    ros2 launch amr_bringup bringup_sim.launch.py use_rviz:=false
    
    # Optional: Use different world
    ros2 launch amr_bringup bringup_sim.launch.py world:=/path/to/world.sdf
    
    # Optional: Use different map
    ros2 launch amr_bringup bringup_sim.launch.py map:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for complete navigation system.
    
    Launch Arguments:
        - world: Path to Gazebo world file
        - map: Path to map YAML file for localization
        - params_file: Path to Nav2 parameters file
        - use_sim_time: Use simulation time (default: true)
        - use_rviz: Launch RViz for navigation visualization (default: true)
        - use_joystick: Enable joystick teleoperation (default: true)
        - x, y, z, yaw: Robot spawn position and orientation
    """
    
    # ========================================
    # Package Directories
    # ========================================
    
    pkg_amr_description = get_package_share_directory('amr_description')
    pkg_amr_teleop = get_package_share_directory('amr_teleop')
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    
    # ========================================
    # Default Paths
    # ========================================
    
    default_world = os.path.join(pkg_amr_description, 'worlds', 'warehouse.sdf')
    default_model = os.path.join(pkg_amr_description, 'urdf', 'robot.xacro')
    default_map_file = os.path.join(pkg_amr_navigation, 'maps', 'my_warehouse_map.yaml')
    default_params_file = os.path.join(pkg_amr_navigation, 'params', 'nav2_params.yaml')
    
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
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to the map YAML file for localization'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the Nav2 parameters YAML file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for navigation visualization'
    )
    
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='true',
        description='Enable joystick teleoperation control'
    )

    use_override_arg = DeclareLaunchArgument(
        'use_teleop_override',
        default_value='true',
        description='Enable teleop override for manual control during navigation'
    )

    teleop_timeout_arg = DeclareLaunchArgument(
        'teleop_timeout',
        default_value='0.5',
        description='Timeout in seconds before switching back to autonomous mode'
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
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_joystick = LaunchConfiguration('use_joystick')
    use_override = LaunchConfiguration('use_teleop_override')
    teleop_timeout = LaunchConfiguration('teleop_timeout')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    
    # ========================================
    # 1. Simulation Launch (Gazebo + Robot)
    # ========================================
    
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_description, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': world,
            'model': model,
            'use_sim_time': use_sim_time,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
            'rviz': 'false',  # We'll use navigation-specific RViz config
            'gui': 'true',
        }.items()
    )
    
    # ========================================
    # 2. Teleoperation Launch
    # ========================================
    
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_teleop, 'launch', 'teleop.launch.py')
        ),
        launch_arguments={
            'use_joystick': use_joystick,
            'cmd_vel_topic': '/cmd_vel_teleop',
        }.items()
    )
    
    # ========================================
    # 3. Navigation Launch (Nav2 + RViz)
    # ========================================
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_navigation, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file,
            'use_rviz': use_rviz,
            'use_teleop_override': use_override,
            'teleop_timeout': teleop_timeout,
        }.items()
    )
    
    # ========================================
    # Launch Description Assembly
    # ========================================
    
    return LaunchDescription([
        # Launch arguments
        world_arg,
        model_arg,
        map_arg,
        params_file_arg,
        use_sim_time_arg,
        use_rviz_arg,
        use_joystick_arg,
        use_override_arg,
        teleop_timeout_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        
        # Launch files in order
        simulation_launch,
        teleop_launch,
        navigation_launch,
    ])
