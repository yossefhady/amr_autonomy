#!/usr/bin/env python3
"""
Unified launch file for simulating the warehouse AGV robot in Gazebo with RViz visualization.
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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for complete simulation with visualization."""
    
    # Get package directories
    pkg_description = get_package_share_directory('amr_description')
    
    # Paths
    default_model_path = os.path.join(pkg_description, 'urdf', 'robot.xacro')
    default_world_path = os.path.join(pkg_description, 'worlds', 'warehouse.sdf')
    default_rviz_config_path = os.path.join(pkg_description, 'rviz', 'display_config.rviz')
    
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
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file'
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
    
    rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        description='Launch RViz2'
    )
    
    # Get launch configurations
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    rvizconfig = LaunchConfiguration('rvizconfig')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    
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
    
    # Gazebo Sim (gz sim / Gazebo Harmonic)
    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world, '-v', '4'],
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': pkg_description,
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
                    '-x', x,
                    '-y', y,
                    '-z', z,
                    '-Y', yaw
                ],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # ROS-Gazebo Bridge for clock
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
        parameters=[{'use_sim_time': use_sim_time}]
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
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Bridge Camera Image
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Bridge Camera Info (CRITICAL for RViz Camera display)
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_info_bridge',
        output='screen',
        arguments=[
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
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
        parameters=[{'use_sim_time': use_sim_time}]
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
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Bridge joint_states (critical for wheel TF!)
    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_states_bridge',
        output='screen',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Bridge TF (for proper coordinate frame synchronization)
    bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        output='screen',
        arguments=[
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
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
                arguments=['-d', rvizconfig],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(rviz)
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        model_arg,
        world_arg,
        rviz_config_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        use_sim_time_arg,
        gui_arg,
        rviz_arg,
        
        # Core nodes
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot,
        
        # Bridges
        bridge_clock,
        bridge_lidar,
        bridge_imu,
        bridge_camera,
        bridge_camera_info,
        bridge_cmd_vel,
        bridge_odom,
        bridge_joint_states,
        bridge_tf,
        
        # Visualization
        rviz_node,
    ])
