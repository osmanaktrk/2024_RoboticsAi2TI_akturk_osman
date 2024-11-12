#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class Node1(Node):
    def __init__(self):
        super().__init__("demo1_node1")
        self.counter = 0
        self.get_logger().info("Hello Osman")
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter+=1
        self.get_logger().info(f"Hello {self.counter}")



def main(args = None):
    rclpy.init(args=args)
    node = Node1()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

