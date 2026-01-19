#!/usr/bin/env python3
"""
Navigation Bringup Launch File for AMR Robot
============================================
Central launch file for Autonomous Navigation operations.
It launches:
1. The Simulation (amr_description/sim.launch.py)
2. EKF Localization (amr_navigation/ekf.launch.py)
3. Navigation Stack (amr_navigation/nav.launch.py)
4. Teleop Node (joystick/keyboard)
5. Teleop Override Node (safety mux)

Modes:
- normal: Standard Pure Pursuit controller
- mppi: MPPI controller (requires additional config)

Usage Examples:
---------------
1. Start Navigation (Default):
    ros2 launch amr_bringup nav_bringup.launch.py

2. Start Navigation in a specific world/map:
    ros2 launch amr_bringup nav_bringup.launch.py world:=medium_warehouse map:=medium_warehouse

3. Start with MPPI controller:
    ros2 launch amr_bringup nav_bringup.launch.py mode:=mppi
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    pkg_amr_description = get_package_share_directory('amr_description')
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    pkg_amr_teleop = get_package_share_directory('amr_teleop')

    # Default paths
    default_nav_rviz_config = os.path.join(pkg_amr_navigation, 'rviz', 'nav2_config.rviz')

    # ========================================
    # Launch Arguments
    # ========================================

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='normal',
        description='Navigation mode: normal (Pure Pursuit) or advanced (MPPI)'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_nav_rviz_config,
        description='Full path to the RVIZ config file to use')

    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='true',
        description='Enable joystick teleoperation'
    )
    
    joystick_device_arg = DeclareLaunchArgument(
        'joystick_device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='medium_warehouse',
        description='World name for simulation (e.g., medium_warehouse)'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='medium_warehouse',
        description='Map name for navigation (e.g., medium_warehouse)'
    )

    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Initial X position'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Initial Y position'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial Yaw orientation'
    )

    # Launch configurations
    world_name = LaunchConfiguration('world')
    map_name = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')
    rviz_config = LaunchConfiguration('rviz_config')
    use_joystick = LaunchConfiguration('use_joystick')
    joystick_device = LaunchConfiguration('joystick_device')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    # Construct paths - separate for simulation and navigation
    world_sdf_path = PathJoinSubstitution([pkg_amr_description, 'worlds', [world_name, '.sdf']])
    map_yaml_path = PathJoinSubstitution([pkg_amr_navigation, 'maps', map_name, [map_name, '.yaml']])

    # ========================================
    # 1. Simulation Launch
    # ========================================

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_description, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': world_sdf_path,
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config,
            'drive_mode': 'diff',
            'x': x,
            'y': y,
            'yaw': yaw
        }.items()
    )

    # ========================================
    # 2. EKF Localization Launch
    # ========================================

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_navigation, 'launch', 'ekf.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ========================================
    # 3. Navigation Launch
    # ========================================

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_navigation, 'launch', 'nav.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_path,
            'mode': mode,
            'x': x,
            'y': y,
            'yaw': yaw
        }.items()
    )

    # ========================================
    # 4. Teleop Launch
    # ========================================
    
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_teleop, 'launch', 'teleop.launch.py')
        ),
        launch_arguments={
            'use_joystick': use_joystick,
            'device': joystick_device,
            'cmd_vel_topic': '/cmd_vel_teleop',
        }.items()
    )

    # ========================================
    # 5. Teleop Override Node
    # ========================================
    
    teleop_override_node = Node(
        package='amr_navigation',
        executable='teleop_override',
        name='teleop_override',
        output='screen',
        parameters=[{
            'teleop_timeout': 0.5,
            'teleop_deadzone': 0.01,
            'smooth_transition': True,
            'override_priority': 'teleop'
        }]
    )

    return LaunchDescription([
        world_arg,
        map_arg,
        use_sim_time_arg,
        mode_arg,
        rviz_config_arg,
        use_joystick_arg,
        joystick_device_arg,
        x_arg,
        y_arg,
        yaw_arg,

        sim_launch,
        ekf_launch,
        nav_launch,
        teleop_launch,
        teleop_override_node
    ])
