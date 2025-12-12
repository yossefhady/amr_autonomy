#!/usr/bin/env python3
"""
SLAM Bringup Launch File for AMR Robot
======================================
Central launch file for SLAM operations.
It launches:
1. The Simulation (amr_description/sim.launch.py)
2. The SLAM Node (based on 'mode')
3. RViz (with SLAM configuration)

Modes:
- mapping: Start fresh mapping (slam.launch.py)
- continue_mapping: Continue previous map (continue_mapping.launch.py)
- localization: Localization only (localization.launch.py)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Package directories
    pkg_amr_slam = get_package_share_directory('amr_slam')
    pkg_amr_description = get_package_share_directory('amr_description')
    pkg_amr_teleop = get_package_share_directory('amr_teleop')
    
    # Default paths
    default_rviz_config = os.path.join(pkg_amr_slam, 'rviz', 'slam_view.rviz')
    
    # ========================================
    # Launch Arguments
    # ========================================
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='SLAM mode: mapping, continue_mapping, or localization'
    )
    
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='medium_warehouse',
        description='World name (for simulation and map loading)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz config file'
    )

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
    
    # Configurations
    mode = LaunchConfiguration('mode')
    world_name = LaunchConfiguration('world_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    use_joystick = LaunchConfiguration('use_joystick')
    joystick_device = LaunchConfiguration('joystick_device')
    
    # ========================================
    # 1. Simulation Launch
    # ========================================
    
    # We launch the simulation, but we override the RViz config to use our SLAM view
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_description, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': PathJoinSubstitution([pkg_amr_description, 'worlds', [world_name, '.sdf']]),
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config,
            'drive_mode': 'mecanum',
        }.items()
    )
    
    # ========================================
    # 2. SLAM Launch (Conditional)
    # ========================================
    
    # Mode: Mapping (Fresh Start)
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_slam, 'launch', 'slam.launch.py')
        ),
        condition=IfCondition(EqualsSubstitution(mode, 'mapping')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Mode: Continue Mapping
    continue_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_slam, 'launch', 'continue_mapping.launch.py')
        ),
        condition=IfCondition(EqualsSubstitution(mode, 'cont')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_name': world_name,
        }.items()
    )
    
    # Mode: Localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_slam, 'launch', 'localization.launch.py')
        ),
        condition=IfCondition(EqualsSubstitution(mode, 'loc')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_name': world_name,
        }.items()
    )
    
    # ========================================
    # 3. Teleop Launch
    # ========================================
    
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_teleop, 'launch', 'teleop.launch.py')
        ),
        launch_arguments={
            'use_joystick': use_joystick,
            'device': joystick_device,
        }.items()
    )
    return LaunchDescription([
        mode_arg,
        world_name_arg,
        use_sim_time_arg,
        rviz_config_arg,
        use_joystick_arg,
        joystick_device_arg,
        
        sim_launch,
        mapping_launch,
        continue_mapping_launch,
        localization_launch,
        teleop_launch,
    ])
