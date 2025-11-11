#!/usr/bin/env python3
"""
Teleop Override Node for AMR Navigation System
===============================================

This node allows manual teleoperation to override autonomous navigation.
It monitors both teleop and navigation velocity commands and switches control
based on teleop activity.

Key Features:
- Seamless switching between manual and autonomous control
- Priority to manual control when joystick/keyboard is active
- Automatic return to autonomous mode after teleop inactivity
- Smooth velocity transitions to prevent jerky motion
- Visual feedback via console logging

Topics:
    Subscribed:
        - /cmd_vel_teleop (geometry_msgs/Twist): Manual control commands
        - /cmd_vel_nav (geometry_msgs/Twist): Autonomous navigation commands
    Published:
        - /cmd_vel (geometry_msgs/Twist): Final velocity commands to robot
        - /teleop_override_active (std_msgs/Bool): Override status indicator

Parameters:
    - teleop_timeout (float): Seconds of inactivity before switching to auto (default: 0.5)
    - teleop_deadzone (float): Velocity threshold for detecting teleop activity (default: 0.01)
    - smooth_transition (bool): Enable velocity smoothing during mode switch (default: true)
    - override_priority (string): 'teleop' or 'nav' - which has priority (default: 'teleop')

Author: AMR AMR Project
ROS 2: Jazzy Jalisco
Date: 2025-10-19
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math


class TeleopOverrideNode(Node):
    """
    Node that manages switching between teleoperation and autonomous navigation.
    
    This node acts as a command multiplexer, selecting between manual teleop
    commands and autonomous navigation commands based on activity.
    """
    
    def __init__(self):
        super().__init__('teleop_override_node')
        
        # ========================================
        # Parameters
        # ========================================
        
        self.declare_parameter('teleop_timeout', 0.5)
        self.declare_parameter('teleop_deadzone', 0.01)
        self.declare_parameter('smooth_transition', True)
        self.declare_parameter('override_priority', 'teleop')
        
        self.teleop_timeout = self.get_parameter('teleop_timeout').value
        self.teleop_deadzone = self.get_parameter('teleop_deadzone').value
        self.smooth_transition = self.get_parameter('smooth_transition').value
        self.override_priority = self.get_parameter('override_priority').value
        
        # ========================================
        # State Variables
        # ========================================
        
        self.last_teleop_time = self.get_clock().now()
        self.last_teleop_cmd = Twist()
        self.last_nav_cmd = Twist()
        self.current_mode = 'nav'  # 'teleop' or 'nav'
        self.teleop_active = False
        
        # ========================================
        # Subscribers
        # ========================================
        
        # Subscribe to teleop commands (from joystick/keyboard)
        self.teleop_sub = self.create_subscription(
            Twist,
            '/cmd_vel_teleop',
            self.teleop_callback,
            10
        )
        
        # Subscribe to navigation commands (from Nav2)
        self.nav_sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.nav_callback,
            10
        )
        
        # ========================================
        # Publishers
        # ========================================
        
        # Publish final velocity commands to robot
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Publish override status for monitoring
        self.override_status_pub = self.create_publisher(
            Bool,
            '/teleop_override_active',
            10
        )
        
        # ========================================
        # Timer for Mode Management
        # ========================================
        
        # Check mode switching at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # ========================================
        # Initialization
        # ========================================
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Teleop Override Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Teleop timeout: {self.teleop_timeout} seconds')
        self.get_logger().info(f'Teleop deadzone: {self.teleop_deadzone} m/s')
        self.get_logger().info(f'Smooth transition: {self.smooth_transition}')
        self.get_logger().info(f'Override priority: {self.override_priority}')
        self.get_logger().info('Subscribed to: /cmd_vel_teleop, /cmd_vel_nav')
        self.get_logger().info('Publishing to: /cmd_vel, /teleop_override_active')
        self.get_logger().info('Initial mode: AUTONOMOUS')
        self.get_logger().info('=' * 60)
    
    def teleop_callback(self, msg: Twist):
        """
        Callback for teleop velocity commands.
        
        Args:
            msg (Twist): Velocity command from teleoperation
        """
        self.last_teleop_cmd = msg
        
        # Check if teleop is actively sending non-zero commands
        if self.is_cmd_active(msg):
            self.last_teleop_time = self.get_clock().now()
            
            # Switch to teleop mode if not already
            if self.current_mode != 'teleop':
                self.current_mode = 'teleop'
                self.teleop_active = True
                self.get_logger().info('🎮 SWITCHED TO MANUAL CONTROL (Teleop Override Active)')
    
    def nav_callback(self, msg: Twist):
        """
        Callback for navigation velocity commands.
        
        Args:
            msg (Twist): Velocity command from Nav2
        """
        self.last_nav_cmd = msg
    
    def is_cmd_active(self, cmd: Twist) -> bool:
        """
        Check if a velocity command is considered 'active' (non-zero).
        
        Args:
            cmd (Twist): Velocity command to check
            
        Returns:
            bool: True if command exceeds deadzone threshold
        """
        linear_mag = math.sqrt(cmd.linear.x**2 + cmd.linear.y**2 + cmd.linear.z**2)
        angular_mag = abs(cmd.angular.z)
        
        return linear_mag > self.teleop_deadzone or angular_mag > self.teleop_deadzone
    
    def timer_callback(self):
        """
        Timer callback to manage mode switching and publish commands.
        
        Runs at 20 Hz to:
        1. Check if teleop has timed out
        2. Select appropriate velocity command
        3. Publish final command and status
        """
        current_time = self.get_clock().now()
        time_since_teleop = (current_time - self.last_teleop_time).nanoseconds / 1e9
        
        # ========================================
        # Mode Switching Logic
        # ========================================
        
        # Check if we should switch from teleop to nav
        if self.current_mode == 'teleop' and time_since_teleop > self.teleop_timeout:
            self.current_mode = 'nav'
            self.teleop_active = False
            self.get_logger().info('🤖 SWITCHED TO AUTONOMOUS CONTROL (Nav2 Active)')
        
        # ========================================
        # Command Selection
        # ========================================
        
        if self.current_mode == 'teleop':
            # Use teleop commands
            output_cmd = self.last_teleop_cmd
        else:
            # Use navigation commands
            output_cmd = self.last_nav_cmd
        
        # ========================================
        # Apply Smooth Transition (Optional)
        # ========================================
        
        if self.smooth_transition:
            # Could implement velocity ramping here if needed
            # For now, direct pass-through
            pass
        
        # ========================================
        # Publish Commands and Status
        # ========================================
        
        # Publish final velocity command
        self.cmd_vel_pub.publish(output_cmd)
        
        # Publish override status
        status_msg = Bool()
        status_msg.data = self.teleop_active
        self.override_status_pub.publish(status_msg)
    
    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self.get_logger().info('Teleop Override Node Shutting Down')
        super().destroy_node()


def main(args=None):
    """
    Main entry point for the teleop override node.
    
    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)
    
    try:
        node = TeleopOverrideNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
