#!/usr/bin/env python3
"""
Launch file for simulating the warehouse AGV robot in Gazebo.
This file launches:
  - Gazebo simulator (gz sim / Gazebo Harmonic)
  - robot_state_publisher: publishes the robot's TF transforms
  - spawn entity: spawns the robot in Gazebo
  - ROS-Gazebo bridge: bridges topics between ROS 2 and Gazebo
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for Gazebo simulation."""
    
    # Get package directories
    pkg_description = get_package_share_directory('avg_description')
    
    # Paths
    default_model_path = os.path.join(pkg_description, 'urdf', 'robot.xacro')
    default_world_path = os.path.join(pkg_description, 'worlds', 'warehouse.sdf')
    
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
    
    x_arg = DeclareLaunchArgument(
        name='x',
        default_value='0.0',
        description='X coordinate for robot spawn position'
    )
    
    y_arg = DeclareLaunchArgument(
        name='y',
        default_value='0.0',
        description='Y coordinate for robot spawn position'
    )
    
    z_arg = DeclareLaunchArgument(
        name='z',
        default_value='0.1',
        description='Z coordinate for robot spawn position'
    )
    
    yaw_arg = DeclareLaunchArgument(
        name='yaw',
        default_value='0.0',
        description='Yaw angle for robot spawn orientation'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Launch Gazebo GUI'
    )
    
    # Get launch configurations
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    
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
    
    # Gazebo Sim (gz sim / Ignition Gazebo / Gazebo Harmonic)
    # Start Gazebo with the specified world
    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world, '-v', '4'],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': pkg_description}
    )
    
    # Alternative if using older Ignition Gazebo naming
    # gazebo_launch = ExecuteProcess(
    #     cmd=['ign', 'gazebo', '-r', world, '-v', '4'],
    #     output='screen',
    #     additional_env={'IGN_GAZEBO_RESOURCE_PATH': pkg_description}
    # )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name', 'warehouse_agv',
            '-topic', '/robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ROS-Gazebo Bridge for topics
    # Bridge clock
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Bridge LIDAR scan
    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/scan', '/scan')
        ]
    )
    
    # Bridge IMU
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        output='screen',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/imu', '/imu')
        ]
    )
    
    # Bridge Camera
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw')
        ]
    )
    
    # Bridge cmd_vel for robot control
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel')
        ]
    )
    
    # Bridge odometry
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        output='screen',
        arguments=[
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/odom', '/odom')
        ]
    )
    
    return LaunchDescription([
        # Arguments
        model_arg,
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        use_sim_time_arg,
        gui_arg,
        
        # Nodes
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot,
        
        # Bridges
        bridge_clock,
        bridge_lidar,
        bridge_imu,
        bridge_camera,
        bridge_cmd_vel,
        bridge_odom,
    ])
