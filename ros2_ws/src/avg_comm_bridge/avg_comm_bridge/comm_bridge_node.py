#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class CommBridgeNode(Node):
    def __init__(self):
        super().__init__('comm_bridge_node')
        self.get_logger().info('Communication Bridge Node has been started')


def main(args=None):
    rclpy.init(args=args)
    node = CommBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
