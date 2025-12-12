#!/usr/bin/env python3
"""
Unified launch file for simulating the warehouse AMR robot in Gazebo with RViz visualization.

Arguments:
    - model: Path to the robot URDF/Xacro file.
    - world: Path to the Gazebo world file.
    - drive_mode: Drive configuration ('diff' or 'mecanum'). Default: 'diff'.
    - rviz: Launch RViz2 (true/false).

This file launches:
    - Gazebo simulator (gz sim / Gazebo Harmonic)
    - robot_state_publisher: publishes the robot's TF transforms
    - Spawn entity: spawns the robot in Gazebo
    - ROS-Gazebo bridge: bridges topics between ROS 2 and Gazebo
    - RViz2: visualization with robot model and sensor data
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for complete simulation with visualization."""
    
    # Get package directories
    pkg_description = get_package_share_directory('amr_description')
    
    # Paths
    default_model_path = os.path.join(pkg_description, 'urdf', 'robot.xacro')
    default_world_path = os.path.join(pkg_description, 'worlds', 'medium_warehouse.sdf')
    default_rviz_config_path = os.path.join(pkg_description, 'rviz', 'display_config.rviz')
    default_models_path = os.path.join(pkg_description, 'models')

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot URDF/Xacro file'
    )
    
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Path to Gazebo world file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    drive_mode_arg = DeclareLaunchArgument(
        name='drive_mode',
        default_value='diff',
        description='Drive mode: diff or mecanum'
    )
    
    # Get launch configurations
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    drive_mode = LaunchConfiguration('drive_mode')
    
    # Process the URDF/Xacro file
    robot_description = ParameterValue(
        Command(['xacro ', model, ' drive_mode:=', drive_mode]),
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
    
    # Gazebo Sim (gz sim / Gazebo Harmonic)
    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world, '-v', '4'],
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': f"{pkg_description}:{default_models_path}",
            # Optional: Enable if using NVIDIA GPU
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'
        }
    )
    
    # Spawn robot in Gazebo (delayed to ensure Gazebo is ready)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-name', 'warehouse_agv',
                    '-topic', '/robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1',
                    '-Y', '0.0'
                ],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # Gazebo Bridges
    gz_bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_states_bridge',
        output='screen',
        arguments=[
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz2 Node (delayed to ensure simulation is running)
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        model_arg,
        world_arg,
        rviz_config_arg,
        use_sim_time_arg,
        drive_mode_arg,
        
        # Core nodes
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot,
        gz_bridges,
        rviz_node,
    ])
