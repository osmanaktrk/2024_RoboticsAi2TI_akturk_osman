import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import random
import time
import math

class  ObstakelForward(Node):

    def __init__(self):
        super().__init__('Obstakel_Real')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT))
        self.timer = None
        self.motion_in_progress = False
        self.cmd = Twist()
        self.laser_forward = 0
        self.range = 0.4
    


        
        
        




    def laser_callback(self,msg):
       
        self.laser_forward = msg.ranges[0]

        self.motion()



    def stop(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.get_logger().info("Stop")
        self.publisher_.publish(self.cmd)


    def move_forward(self):
        if(self.laser_forward>self.range):
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info("Move Forward")
        else:
            self.stop()



    def motion(self):
        if(self.laser_forward < self.range):
            self.stop()
            
        else:

            self.move_forward()
        
     
        
  

            
def main(args=None):
    rclpy.init(args=args)
    node = ObstakelForward()       
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()