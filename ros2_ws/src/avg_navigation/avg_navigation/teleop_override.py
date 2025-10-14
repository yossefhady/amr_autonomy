#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class TeleopOverrideNode(Node):
    def __init__(self):
        super().__init__('teleop_override_node')
        self.get_logger().info('Teleop Override Node has been started')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopOverrideNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
