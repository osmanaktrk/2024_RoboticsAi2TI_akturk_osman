import cv2
import mediapipe as mp

# Initialize MediaPipe Hand solution
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, 
                       max_num_hands=2, 
                       min_detection_confidence=0.5, 
                       min_tracking_confidence=0.5)
mp_draw = mp.solutions.drawing_utils

# Open the webcam
cap = cv2.VideoCapture(0)

# Define finger tip landmarks (based on MediaPipe hand model)
finger_tips = [4, 8, 12, 16, 20]  # Thumb, Index, Middle, Ring, Little finger tips
finger_base = [3, 6, 10, 14, 18]  # Base points to compare for open/closed status

print("Press 'q' to exit the application.")
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame, exiting...")
        break

    # Flip the frame horizontally for a natural mirror view
    frame = cv2.flip(frame, 1)

    # Convert the frame to RGB (MediaPipe expects RGB)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process the frame to detect hands
    results = hands.process(rgb_frame)

    # Check if hands are detected
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Draw the hand landmarks on the frame
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Get the height and width of the frame
            h, w, _ = frame.shape

            # Store the status of fingers
            finger_status = []

            for tip, base in zip(finger_tips, finger_base):
                # Get coordinates of the fingertip and its base
                tip_y = hand_landmarks.landmark[tip].y * h
                base_y = hand_landmarks.landmark[base].y * h

                # Determine if the finger is open (tip above base)
                finger_status.append(tip_y < base_y)

            # Display the status of fingers on the frame
            for i, status in enumerate(finger_status):
                cv2.putText(frame, f"Finger {i + 1}: {'Open' if status else 'Closed'}",
                            (10, 30 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("Finger Tracking", frame)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources and close windows
cap.release()
cv2.destroyAllWindows()
