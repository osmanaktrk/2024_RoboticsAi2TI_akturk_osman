import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import random

class  Obstakel(Node):

    def __init__(self):
         # Here you have the class constructor
        # call the class constructor
        super().__init__('obstakel')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_forward = 0
        self.laser_frontLeft = 0
        self.laser_frontRight = 0
        # create a Twist message
        self.cmd = Twist()
        
        self.timer = self.create_timer(self.timer_period, self.motion)




    def laser_callback(self,msg):
        # Save the frontal laser scan info
        self.laser_frontLeft = min(msg.ranges[0:15])
        self.laser_frontRight = min(msg.ranges[349:359])
        self.laser_forward = min(self.laser_frontLeft, self.laser_frontRight)


        
    def motion(self):
        
        # Logic of move

        if(self.laser_forward < 0.5):
            #stop
            self.cmd.linear.x = 0.0

            if(self.laser_frontLeft<0.5 and self.laser_frontRight>0.5):
                #turn right
                self.cmd.angular.z = -1*random.randint(1, 20)/10
                self.cmd.linear.x = 0.3

            elif(self.laser_frontLeft>0.5 and self.laser_frontRight<0.5):
                #turn left
                self.cmd.angular.z = random.randint(1, 20)/10
                self.cmd.linear.x = 0.3

            else:
                #turn right fix
                self.cmd.angular.z = -0.5

        else:
            self.cmd.linear.x = 0.3
            self.cmd.angular.z = 0.0
        
        # Publishing the cmd_vel values to a Topic
        self.publisher_.publish(self.cmd)
     
        
  

            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    obstakel = Obstakel()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(obstakel)
    # Explicity destroy the node
    obstakel.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()