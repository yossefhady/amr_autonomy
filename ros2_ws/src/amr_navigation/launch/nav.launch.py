import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # Default paths
    default_params_file = os.path.join(pkg_amr_navigation, 'params', 'nav2_params.yaml')
    default_rviz_config = os.path.join(pkg_amr_navigation, 'rviz', 'nav2_config.rviz')
    default_map_file = os.path.join(pkg_amr_navigation, 'maps', 'my_warehouse_map.yaml') 

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ')

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RVIZ config file to use')

    # Launch Nav2 Bringup
    # This launches map_server, amcl, planner, controller, recoveries, etc.
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_yaml_file,
            'autostart': 'true',
            'use_composition': 'True', # Uses component containers for efficiency
        }.items()
    )

    # RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_map_yaml_cmd,
        declare_use_rviz_cmd,
        declare_rviz_config_cmd,
        nav2_bringup_launch,
        rviz_node
    ])