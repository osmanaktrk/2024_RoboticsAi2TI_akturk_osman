import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import random
import time
import math

class  ObstakelReal(Node):

    def __init__(self):
        super().__init__('Obstakel_Real')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT))
        self.timer = None
        self.motion_in_progress = False
        self.cmd = Twist()
        self.laser_forwardArea = 0
        self.laser_frontLeft_30 = 0
        self.laser_frontRight_30 = 0
        self.laser_forward = 0
        self.range = 0.4

        
        
        




    def laser_callback(self,msg):
        self.laser_frontLeft_30 = min(msg.ranges[:40])
       
        self.laser_frontRight_30 = min(msg.ranges[-40:])
             
        
        self.laser_forwardArea = min(msg.ranges[-40:] + msg.ranges[:40])
        self.laser_forward = msg.ranges[0]

        if not self.motion_in_progress:
            self.motion()


    def stop(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.get_logger().info("Stop")
        self.publisher_.publish(self.cmd)


    def move_forward(self):
        if(self.laser_forward>self.range):
            self.cmd.linear.x = 0.3
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info("Move Forward")
        else:
            self.stop()


    
    def turn(self, direction):
        self.motion_in_progress = True
        self.cmd.linear.x = 0.0
        if(direction == 'l'):
            self.cmd.angular.z = 0.3
            self.get_logger().info("Turn Left")
        elif(direction == 'r'):
            self.cmd.angular.z = -0.3
            self.get_logger().info("Turn Right")
        self.publisher_.publish(self.cmd)
        self.timer = self.create_timer(random.uniform(0.1, 1.0), self.stop_turn)
        
    

    def turn_corner(self, direction):
        self.motion_in_progress = True
        self.cmd.linear.x = 0.0
        if(direction == 'l'):
            self.cmd.angular.z = 0.3
            self.get_logger().info("Corner Turn Left")
        elif(direction == 'r'):
            self.cmd.angular.z = -0.3
            self.get_logger().info("Corner Turn Right")
        self.publisher_.publish(self.cmd)
        self.timer = self.create_timer(random.uniform(3.0, 7.0), self.stop_turn)
        



    def stop_turn(self):
        self.stop()
        self.motion_in_progress = False
        if self.timer:
            self.timer.cancel()
            self.timer = None

    def motion(self):
        if(self.laser_forwardArea < self.range):
            self.stop()
            
           

            if(self.laser_frontRight_30 > self.range and self.laser_frontRight_30 > self.laser_frontLeft_30):
                self.turn('r')
            elif(self.laser_frontLeft_30 > self.range and self.laser_frontLeft_30 > self.laser_frontRight_30):
                self.turn('l')

       

            else:
                if (self.laser_frontLeft_30 > self.laser_frontRight_30):
                    self.turn_corner('l')     
                else:
                    self.turn_corner('r') 
        else:

            self.move_forward()
        
     
        
  

            
def main(args=None):
    rclpy.init(args=args)
    node = ObstakelReal()       
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()