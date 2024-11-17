import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class Publisher(Node):
    def __init__(self):
        super().__init__("number_publisher")
        
        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.timer_ = self.create_timer(0.5, self.callback_number_publisher)
        self.get_logger().info("Number publisher has been started")

    def callback_number_publisher(self):
        msg = Int64()
        msg.data = 5
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
