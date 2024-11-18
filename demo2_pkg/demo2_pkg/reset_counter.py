import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class Reset(Node):
    def __init__(self):
        super().__init__("reset_counter")

        self.client_ = self.create_client(SetBool, "reset_counter")

        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the service")

        request = SetBool.Request()
        request.data = True

        self.future = self.client_.call_async(request)
        self.future.add_done_callback(self.callback_response)



    def callback_response(self, future):
        try:
            response = future.result()

            if response.success:
                self.get_logger().info(f"Service response: {response.message}")
            else:
                self.get_logger().warn(f"Service failed: {response.message}")

            rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")






def main(args=None):
    rclpy.init()
    node=Reset()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()