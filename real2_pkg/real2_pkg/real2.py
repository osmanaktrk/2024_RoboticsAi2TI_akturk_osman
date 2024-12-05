import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import time
import random
import math

class Real2(Node):
    def __init__(self):
        super().__init__("Real2")
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.timer_period = 1  # Timer period in seconds
        self.blocked_since = None  # Timestamp when the robot got blocked
        self.laser_forward = float('inf')
        self.laser_frontLeft = float('inf')
        self.laser_frontRight = float('inf')
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)



    def laser_callback(self, msg):
        self.laser_forward = msg.ranges[359]
        self.laser_frontLeft = min(msg.ranges[0:20])
        self.laser_frontRight = min(msg.ranges[340:359])

    def move(self, distance):
       # Log the orientation (linear_velocity)
        self.get_logger().info(f'Moving with distance: {distance} m')

        
        self.cmd.linear.x = 0.1
        self.cmd.angular.z = 0.0
    

        #To divide the distance into a certain speed and apply it as a time
        duration = distance/0.1


        start_time = time.time()

        while time.time() - start_time < duration:

            if self.laser_forward > 0.5:
                self.publisher_.publish(self.cmd)
            else:
                self.cmd.linear.x = 0.0
                self.get_logger().warn("I can not go forward")
            
            
            time.sleep(0.01)  # sleep for 10ms

        # Stop the robot
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)

    def turn(self, angle):
        # Log the orientation (angle)
        self.get_logger().info(f'Moving with angle: {angle}')

        self.cmd.linear.x = 0.0

        if angle < 0:
            self.cmd.angular.z = 0.2
        else:
            self.cmd.angular.z = -0.2

        start_time = time.time()

        duration = math.pi/((180/abs(angle))*abs(self.cmd.angular.z))
        self.get_logger().info(f"duration: {duration}")

        while time.time() - start_time < duration:
            self.publisher_.publish(self.cmd)
            time.sleep(0.01)  # sleep for 10ms

        # Stop the robot
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)


    def motion(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        
        self.move(1.9)
        self.turn(90)
        self.move(1.2)
        self.turn(-90)
        self.move(1.9)
        self.turn(90)
        self.move(1.1)
        self.turn(-90)
        self.motion(0.3)




def main(args=None):
    rclpy.init(args=args)
    node = Real2()
    node.motion()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()