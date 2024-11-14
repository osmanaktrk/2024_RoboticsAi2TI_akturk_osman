#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class Publisher1(Node):
    def __init__(self):
        super.__init__("demo1_publisher1")
        



def main(args=None):
    rclpy.init(args=args)
    node=Publisher1()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
