import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import time



class Real1(Node):
    def __init__(self):
        super().__init__("real1")
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer_period = 0.5
        self.laser_forward = 0
        self.laser_frontLeft = 0
        self.laser_frontRight = 0
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)


    def laser_callback(self,msg): 
        self.laser_forward = msg.ranges[0] 
        #self.laserResolution = 360/len(msg.ranges)
        #self.laser_frontLeft = min(msg.ranges[0:25]) 
        #self.laser_frontRight = min(msg.ranges[-25:])
        #left90 = int(90/self.laserResolution) 
        #self.laser_left = min(msg.ranges[left90]) 



    def motion(self):
        

        if(self.laser_forward < 0.5):
            self.cmd.linear.x = 0.0
        else:
            self.cmd.linear.x = 0.3
            self.cmd.angular.z = 0.0
  
        self.publisher_.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Real1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
