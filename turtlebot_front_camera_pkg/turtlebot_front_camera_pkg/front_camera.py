import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FrontCameraViewer(Node):
    def __init__(self):
        super().__init__('front_camera_viewer')
        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Camera topic name
            self.image_callback,
            10)
        self.bridge = CvBridge()  # Used to convert ROS Image messages to OpenCV format
        self.get_logger().info('Front Camera Viewer Node started.')

    def image_callback(self, msg):
        try:
            # Convert ROS 2 Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            scale_percent = 30  # Resize to 50% of the original size
            width = int(cv_image.shape[1] * scale_percent / 100)
            height = int(cv_image.shape[0] * scale_percent / 100)

            # Resize the image to the desired dimensions (e.g., 800x600)
            resized_image = cv2.resize(cv_image, (width, height))


            
            
            # Display the resized image in a new window
            cv2.imshow('Front Camera View', resized_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FrontCameraViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
