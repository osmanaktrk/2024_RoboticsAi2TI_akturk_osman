

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import cv2
import mediapipe as mp
import math

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        self.max_speed = 0.15
        self.min_speed = 0.0

        # ROS 2 Publishers
        self.speed_publisher = self.create_publisher(Float32, 'robot_speed', 10)
        self.stop_publisher = self.create_publisher(Bool, 'robot_stop', 10)
        
        # Mediapipe initialization
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils
        
        # OpenCV Camera Initialization
        self.cap = cv2.VideoCapture(0)
        
        # Timer to process the camera feed
        self.timer = self.create_timer(0.1, self.process_camera)

    def process_camera(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Camera frame not received.')
            return

        # Convert frame to RGB for Mediapipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        speed = 0.0
        stop = True

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                if self.is_fist(hand_landmarks):
                    stop = True  # Robot stops
                    speed = 0.0
                    self.get_logger().info('Fist detected. Robot stopped.')
                else:
                    stop = False  # Robot starts moving
                    speed = self.calculate_speed(hand_landmarks)
                    self.get_logger().info(f'Open hand detected. Calculated speed: {speed:.2f}')

        # Publish speed and stop signals
        self.speed_publisher.publish(Float32(data=speed))
        self.stop_publisher.publish(Bool(data=stop))

        # Display speed and stop status on the frame
        cv2.putText(frame, f'Speed: {speed:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(frame, f'Stop: {stop}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show the video frame
        cv2.imshow("Hand Control", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()

    def is_fist(self, hand_landmarks):
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]

        thumb_distance = math.sqrt((thumb_tip.x - wrist.x)**2 + (thumb_tip.y - wrist.y)**2)
        index_distance = math.sqrt((index_tip.x - wrist.x)**2 + (index_tip.y - wrist.y)**2)

        return thumb_distance < 0.1 and index_distance < 0.1

    def calculate_speed(self, hand_landmarks):
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

        

        # Calculate distance between thumb and index finger
        distance = math.sqrt((thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2)
        # Apply threshold to avoid very small non-zero values
        if distance < 0.04:
            distance = 0.0
        # Map distance to speed (min to max)
        return max(self.min_speed, min(self.max_speed, distance))


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

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



