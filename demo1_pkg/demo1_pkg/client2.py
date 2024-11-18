import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial


class Client2(Node):
    def __init__(self):
        super().__init__("Client2")
        self.call_add_two_ints(5, 6)
        self.call_add_two_ints(6, 6)
        self.call_add_two_ints(7, 6)



    
    def call_add_two_ints(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service")
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))

    def callback_call_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(f"{a} + {b} = {response.sum}")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))




def main(args=None):
    rclpy.init(args=args)
    node = Client2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
