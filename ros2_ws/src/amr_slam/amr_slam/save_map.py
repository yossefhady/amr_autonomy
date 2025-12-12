#!/usr/bin/env python3
"""
Map Saver Script for AMR Robot
==============================
Saves the map in two locations for different purposes:
1. amr_slam/maps/<world>/<world>: Serialized format (.posegraph/.data) for continuing mapping.
2. amr_navigation/maps/<world>/<world>: Standard format (.pgm/.yaml) for AMCL localization.

Usage: ros2 run amr_slam save_map.py <world_name>
Example: ros2 run amr_slam save_map.py medium_warehouse
"""

import sys
import os
import subprocess
import pathlib
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SerializePoseGraph
from ament_index_python.packages import get_package_share_directory

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver_client')
        self.cli_serialize = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        
        # We don't wait indefinitely here to allow the script to fail fast if SLAM isn't running
        if not self.cli_serialize.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Service /slam_toolbox/serialize_map not available.')
            self.get_logger().warn('Is slam_toolbox running? Serialized map will NOT be saved.')
            self.service_available = False
        else:
            self.service_available = True
            
        self.req_serialize = SerializePoseGraph.Request()

    def get_src_paths(self):
        """
        Try to locate the source directories to save maps directly to src.
        If not found (e.g. installed binary), fallback to current directory.
        """
        # Use ament_index to find the package, then look for the workspace root
        try:
            # This usually points to install/amr_slam/share/amr_slam
            share_dir = get_package_share_directory('amr_slam')
            path = pathlib.Path(share_dir)
            
            # Traverse up until we find 'install' directory, then go to sibling 'src'
            workspace_root = None
            for parent in path.parents:
                if (parent / 'src').exists() and (parent / 'install').exists():
                    workspace_root = parent
                    break
            
            if workspace_root:
                amr_slam = workspace_root / 'src' / 'amr_slam'
                amr_nav = workspace_root / 'src' / 'amr_navigation'
                if amr_slam.exists() and amr_nav.exists():
                    self.get_logger().info(f"Detected source paths via workspace root: {amr_slam}")
                    return str(amr_slam), str(amr_nav)
        except Exception as e:
            self.get_logger().warn(f"Failed to detect workspace via ament_index: {e}")

        self.get_logger().warn("Could not detect source directory structure. Saving to current directory.")
        return None, None

    def save_serialized_map(self, path_prefix):
        """Saves the internal slam_toolbox state (.posegraph, .data)"""
        if not self.service_available:
            return

        self.get_logger().info(f'Saving serialized map to {path_prefix}...')
        self.req_serialize.filename = path_prefix
        
        future = self.cli_serialize.call_async(self.req_serialize)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            future.result()
            self.get_logger().info('Serialized map saved successfully.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def save_standard_map(self, path_prefix):
        """Saves the standard occupancy grid (.pgm, .yaml) using map_saver_cli"""
        self.get_logger().info(f'Saving standard map to {path_prefix}...')
        
        # Command: ros2 run nav2_map_server map_saver_cli -f <map_name>
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', path_prefix
        ]
        
        try:
            # Run the command and wait for it to finish
            result = subprocess.run(
                cmd, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE, 
                text=True,
                timeout=30
            )
            
            if result.returncode == 0:
                self.get_logger().info('Standard map saved successfully.')
            else:
                self.get_logger().error(f'Failed to save standard map. Error:\n{result.stderr}')
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('Map saver timed out.')
        except FileNotFoundError:
            self.get_logger().error('nav2_map_server not found. Is Nav2 installed?')

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run amr_slam save_map.py <world_name>")
        print("Available worlds: small_warehouse, medium_warehouse, large_warehouse")
        return

    world_name = sys.argv[1]
    
    saver = MapSaver()
    slam_base, nav_base = saver.get_src_paths()
    
    if slam_base and nav_base:
        # Construct paths
        # 1. SLAM Path: src/amr_slam/maps/<world_name>/<world_name>
        slam_map_dir = os.path.join(slam_base, 'maps', world_name)
        slam_map_path = os.path.join(slam_map_dir, world_name)
        
        # 2. Nav Path: src/amr_navigation/maps/<world_name>/<world_name>
        nav_map_dir = os.path.join(nav_base, 'maps', world_name)
        nav_map_path = os.path.join(nav_map_dir, world_name)
        
        # Create directories if they don't exist
        os.makedirs(slam_map_dir, exist_ok=True)
        os.makedirs(nav_map_dir, exist_ok=True)
        
    else:
        # Fallback to current directory
        slam_map_path = world_name
        nav_map_path = world_name

    print(f"Processing world: {world_name}")
    
    # 1. Save Serialized Map (for slam_toolbox)
    saver.save_serialized_map(slam_map_path)
    
    # 2. Save Standard Map (for AMCL/Nav2)
    saver.save_standard_map(nav_map_path)

    saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
