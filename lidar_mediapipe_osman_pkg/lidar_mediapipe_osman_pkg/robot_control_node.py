import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Float32
import random
import math

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        # ROS 2 Publishers and Subscribers
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed_subscriber = self.create_subscription(Float32, 'robot_speed', self.speed_callback, 10)
        self.laser_subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT)
        )

        # Robot movement parameters
        self.cmd = Twist()
        self.speed = 0.0
        self.motion_in_progress = False
        self.timer = None
        self.laser_forwardArea = float('inf')
        self.laser_frontLeft = float('inf')
        self.laser_frontRight = float('inf')
        self.range = 0.4  # Obstacle avoidance range

    def laser_callback(self, msg):
        # Process laser scan data
        self.laser_frontLeft = min(msg.ranges[:40], default=float('inf'))
        self.laser_frontRight = min(msg.ranges[-40:], default=float('inf'))
        self.laser_forwardArea = min(msg.ranges[-40:] + msg.ranges[:40], default=float('inf'))

        if not self.motion_in_progress:
            self.motion()

    def speed_callback(self, msg):
        # Receive speed from CameraNode
        self.speed = msg.data

    def stop(self):
        # Stop the robot
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.get_logger().info("Stop")

    def move_forward(self):
        # Move forward if no obstacle is detected
        if self.laser_forwardArea > self.range:
            self.cmd.linear.x = self.speed
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info(f"Move Forward with speed: {self.speed}")
        else:
            self.stop()

    def turn(self, direction):
        # Turn the robot left or right
        self.motion_in_progress = True
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.3 if direction == 'l' else -0.3
        self.publisher_.publish(self.cmd)
        self.get_logger().info(f"Turn {'Left' if direction == 'l' else 'Right'}")
        self.timer = self.create_timer(random.uniform(0.5, 1.5), self.stop_turn)

    def turn_corner(self, direction):
        # Perform a corner turn
        self.motion_in_progress = True
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.3 if direction == 'l' else -0.3
        self.publisher_.publish(self.cmd)
        self.get_logger().info(f"Corner Turn {'Left' if direction == 'l' else 'Right'}")
        self.timer = self.create_timer(random.uniform(2.0, 5.0), self.stop_turn)

    def stop_turn(self):
        # Stop turning
        self.stop()
        self.motion_in_progress = False
        if self.timer:
            self.timer.cancel()
            self.timer = None

    def motion(self):
        # Decide motion based on laser scan data
        if self.laser_forwardArea < self.range:
            self.stop()

            if self.laser_frontRight > self.range and self.laser_frontRight > self.laser_frontLeft:
                self.turn('r')
            elif self.laser_frontLeft > self.range and self.laser_frontLeft > self.laser_frontRight:
                self.turn('l')
            else:
                if self.laser_frontLeft > self.laser_frontRight:
                    self.turn_corner('l')
                else:
                    self.turn_corner('r')
        else:
            self.move_forward()


def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




