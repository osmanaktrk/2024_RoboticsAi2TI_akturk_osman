import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Service1(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        self.service_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints)
        self.get_logger().info("Service1 has been started")


    def callback_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
        return response



def main(args=None):
    rclpy.init(args=args)
    node = Service1()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()