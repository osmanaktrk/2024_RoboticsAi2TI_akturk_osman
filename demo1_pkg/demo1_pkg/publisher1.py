import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class Publisher1(Node):
    def __init__(self):
        super().__init__("demo1_publisher1")
        self.publisher_ = self.create_publisher(String, "publisher1", 10)
        self.timer_ = self.create_timer(0.5, self.publish_state)
        self.get_logger().info("Publisher has been started.")
        

    def publish_state(self):
        msg = String()
        msg.data = "Hello"
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node=Publisher1()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
