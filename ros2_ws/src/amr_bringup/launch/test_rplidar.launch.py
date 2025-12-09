#!/usr/bin/env python3
"""
Launch file to test RPLidar hardware with RViz visualization.
This launches only the RPLidar node and RViz for quick hardware testing.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar (usually /dev/ttyUSB0 or /dev/ttyUSB1)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Frame ID for the laser scan'
    )
    
    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='Enable angle compensation'
    )
    
    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='Scan mode: Standard, Express, Boost, Sensitivity, Stability'
    )

    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    frame_id = LaunchConfiguration('frame_id')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')

    # RPLidar node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'frame_id': frame_id,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode,
        }]
    )

    # Static transform from base_link to laser frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    # RViz for visualization
    rviz_config_path = os.path.join(
        get_package_share_directory('amr_bringup'),
        'rviz',
        'test_rplidar.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        serial_port_arg,
        frame_id_arg,
        angle_compensate_arg,
        scan_mode_arg,
        rplidar_node,
        static_tf_node,
        rviz_node,
    ])
