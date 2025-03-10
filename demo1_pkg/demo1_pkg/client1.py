import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts




def main(args=None):
    rclpy.init(args=args)
    node = Node("Client1")
    client = node.create_client(AddTwoInts, "add_two_ints")
    node.get_logger().info("Client1 has been created")

    while not client.wait_for_service(1.0):
        node.get_logger().info("Waiting for service")


    request = AddTwoInts.Request()
    request.a = 5
    request.b = 14
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    try:
        response = future.result()
        node.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
    except Exception as e:
        node.get_logger().error("Service call failed %r" % (e,))


    rclpy.shutdown()


if __name__=="__main__":
    main()