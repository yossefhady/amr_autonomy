import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Get Package Directories
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    pkg_amr_description = get_package_share_directory('amr_description')

    # 2. Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')

    # 3. Declare Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock')

    declare_map_file = DeclareLaunchArgument(
        'map',
        # Default to your warehouse map
        default_value=os.path.join(pkg_amr_navigation, 'maps', 'my_warehouse_map.yaml'),
        description='Full path to map file to load')

    # ====================================================
    # 4. LAUNCH SIMULATION (The Body)
    # ====================================================
    # We call your existing 'sim.launch.py' from amr_description.
    # We set 'rviz:=false' because we want to use the Navigation RViz instead.
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_description, 'launch', 'sim.launch.py') 
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': 'false', # Disable Sim RViz to avoid duplicates
            'headless': 'False' # Keep Gazebo GUI open
        }.items()
    )

    # ====================================================
    # 5. LAUNCH NAVIGATION (The Brain)
    # ====================================================
    # We wait 10 seconds for Gazebo to load the robot and TFs.
    # Then we call your 'nav.launch.py'.
    navigation_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_amr_navigation, 'launch', 'nav.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_file,
                    'use_rviz': 'true', # Enable Nav RViz
                    'params_file': os.path.join(pkg_amr_navigation, 'params', 'nav2_params.yaml')
                }.items()
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_file,
        simulation_launch,
        navigation_launch
    ])