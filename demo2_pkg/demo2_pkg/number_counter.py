import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class Counter(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.sum = 0
        self.get_logger().info("Number counter has been started")
        self.create_subscription(Int64, "number", self.callback_number_counter, 10)
        self.publisher_=self.create_publisher(Int64, "Number Sum", 10)


    def callback_number_counter(self, msg):
        self.sum += msg.data
        self.get_logger().info(f" Sum= {self.sum}")

        msg2 = Int64()
        msg2.data = self.sum
        self.publisher_.publish(msg2)


def main(args=None):
    rclpy.init(args=args)
    node=Counter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()




