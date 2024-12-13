import cv2

# Open the webcam
cap = cv2.VideoCapture(0)  # '0' is the default camera. Use other indices for additional cameras.
#cap = cv2.VideoCapture("rtsp://tapoadmin:ailabo123$@10.2.172.155/stream1")  # 'lab' is the default camera. Use other indices for additional cameras.
ret, frame = cap.read()
print("Image size (H, W) is:", frame.shape)



if not cap.isOpened():
    print("Could not open webcam!")
    exit()

print("Webcam is running. Press 'q' to quit.")




while True:
    ret, frame = cap.read()  # Capture a frame from the webcam
    if not ret:
        print("Failed to grab frame, exiting...")
        break

    cv2.imshow("Webcam", frame)  # Display the frame in a window
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Grayscale Webcam", gray_frame)
    
    cv2.imshow("Big Webcam", cv2.resize(frame, None, fx=2, fy=2))  # Display big the frame in a window
    
    cv2.imshow("Small Webcam", cv2.resize(frame, None, fx=0.5, fy=0.5))  # Display small the frame in a window

    cv2.imshow("Cropped Webcam", frame[200:400, 300:600])  # Display small the frame in a window



    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources and close windows
cap.release()
cv2.destroyAllWindows()
