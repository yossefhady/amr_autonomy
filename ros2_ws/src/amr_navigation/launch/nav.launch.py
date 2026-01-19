#!/usr/bin/env python3
"""
Navigation Launch File
======================
Launches the ROS 2 Navigation Stack (Nav2) with custom configuration.
It includes:
1. AMCL (Localization)
2. Planner Server (Global Path Planning)
3. Controller Server (Local Path Following)
4. Behavior Server (Recovery Behaviors)
5. Lifecycle Manager

Modes:
- normal: Uses Regulated Pure Pursuit Controller
- advanced: Uses MPPI Controller (Model Predictive Path Integral)

Usage:
------
ros2 launch amr_navigation nav.launch.py map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import SetRemap, SetParameter
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Default paths
    nav2_params_path = os.path.join(pkg_amr_navigation, 'params', 'nav2_params.yaml')
    mppi_params_path = os.path.join(pkg_amr_navigation, 'params', 'nav2_mppi_params.yaml')
    default_map_path = os.path.join(pkg_amr_navigation, 'maps', 'medium_warehouse', 'medium_warehouse.yaml')

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

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Name of the world (e.g. medium_warehouse)'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic to publish velocity commands'
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

    # Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')
    map_name = LaunchConfiguration('map')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    # ========================================
    # Parameter Rewriting
    # ========================================
    
    # Create a temporary params file with the initial pose injected
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'initial_pose_x': x,
        'initial_pose_y': y,
        'initial_pose_a': yaw,
    }

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_path,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    configured_mppi_params = RewrittenYaml(
        source_file=mppi_params_path,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # ========================================
    # Launch Nav2 Bringup (amcl, planner, controller, etc.)
    # ========================================
    
    # Normal Mode (Pure Pursuit)
    nav2_bringup_normal = GroupAction(
        condition=IfCondition(EqualsSubstitution(mode, 'normal')),
        actions=[
            SetRemap(src='/cmd_vel', dst=cmd_vel_topic),
            # Explicitly set parameters in addition to the params file
            SetParameter(name='initial_pose_x', value=x),
            SetParameter(name='initial_pose_y', value=y),
            SetParameter(name='initial_pose_a', value=yaw),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_name,
                    'use_sim_time': use_sim_time,
                    'params_file': configured_nav2_params,
                    'autostart': 'true',
                    'use_composition': 'False', # Disable composition to ensure parameter propagation
                }.items()
            )
        ]
    )

    # Advanced Mode (MPPI)
    nav2_bringup_advanced = GroupAction(
        condition=IfCondition(EqualsSubstitution(mode, 'mppi')),
        actions=[
            SetRemap(src='/cmd_vel', dst=cmd_vel_topic),
            SetParameter(name='initial_pose_x', value=x),
            SetParameter(name='initial_pose_y', value=y),
            SetParameter(name='initial_pose_a', value=yaw),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_name,
                    'use_sim_time': use_sim_time,
                    'params_file': configured_mppi_params,
                    'autostart': 'true',
                    'use_composition': 'False',
                }.items()
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        mode_arg,
        map_arg,
        cmd_vel_topic_arg,
        x_arg,
        y_arg,
        yaw_arg,
        nav2_bringup_normal,
        nav2_bringup_advanced
    ])
