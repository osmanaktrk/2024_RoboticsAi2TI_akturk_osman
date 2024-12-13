import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Camera topic
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)  # Movement commands topic
        self.bridge = CvBridge()  # Convert ROS Image messages to OpenCV format
        self.get_logger().info('Line Follower Node started.')

    def image_callback(self, msg):
        try:
            # Convert ROS 2 Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize the image to a smaller scale
            scale_percent = 30  # Resize to 30% of the original size
            width = int(frame.shape[1] * scale_percent / 100)
            height = int(frame.shape[0] * scale_percent / 100)
            frame = cv2.resize(frame, (width, height))

            # Display the raw front camera view in a separate window
            cv2.imshow('Front Camera', frame)

            # Convert the image to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define the yellow color range
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])

            # Threshold the HSV image to get only yellow colors
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                # Find the largest contour
                largest_contour = max(contours, key=cv2.contourArea)

                # Get the centroid of the largest contour
                M = cv2.moments(largest_contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])  # x-coordinate of the centroid
                    cy = int(M['m01'] / M['m00'])  # y-coordinate of the centroid

                    # Draw the contour and the centroid
                    cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

                    # Determine movement direction
                    height, width, _ = frame.shape
                    center_x = width // 2

                    twist = Twist()
                    if cx < center_x - 50:
                        self.get_logger().info('Turn Left')
                        twist.angular.z = 0.3  # Rotate left
                    elif cx > center_x + 50:
                        self.get_logger().info('Turn Right')
                        twist.angular.z = -0.3  # Rotate right
                    else:
                        self.get_logger().info('Go Straight')
                        twist.linear.x = 0.2  # Move forward

                    # Publish movement command
                    self.publisher.publish(twist)

            # Show the processed frame with line detection
            cv2.imshow('Line Follower', frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()

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
