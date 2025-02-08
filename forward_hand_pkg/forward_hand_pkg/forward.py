import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import cv2
import mediapipe as mp

class Forward(Node):
    def __init__(self):
        super().__init__("Forward")
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT))
        
        self.cmd = Twist()
        self.speed = 0.0
        self.safe_distance = 0.4
        self.stop_robot = False

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0)

        self.timer = self.create_timer(0.1, self.process_camera)

    def laser_callback(self, msg):
        if min(msg.ranges) < self.safe_distance:
            self.stop_robot = True
        else:
            self.stop_robot = False
        self.control_robot()

    def control_robot(self):
        if self.stop_robot:
            self.cmd.linear.x = 0.0
        else:
            self.cmd.linear.x = self.speed
        self.publisher_.publish(self.cmd)

    def process_camera(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
                index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                if self.is_fist(hand_landmarks):
                    self.speed = 0.0
                else:
                    self.speed = self.calculate_speed(thumb_tip, index_tip)

        cv2.imshow("Hand Control", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()

    def is_fist(self, hand_landmarks):
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        return abs(thumb_tip.x - index_tip.x) < 0.05

    def calculate_speed(self, thumb_tip, index_tip):
        distance = math.sqrt((thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2)
        return max(0.0, min(0.2, distance))

def main(args=None):
    rclpy.init(args=args)
    node = Forward()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
