import cv2
import numpy as np

# Initialize the camera (0 for default camera)
cap = cv2.VideoCapture(0)

# Initialize color flags
red_flag, green_flag, blue_flag, yellow_flag = False, False, False, False
red_color, green_color, blue_color, yellow_color = 0, 0, 0, 0

def find_dominant_color(frame):
    global red_flag, green_flag, blue_flag, red_color, green_color, blue_color

    b, g, r = cv2.split(frame)
    b_mean, g_mean, r_mean = np.mean(b), np.mean(g), np.mean(r)

    if not (red_flag or blue_flag or yellow_flag):  # Only check if neither red nor blue nor yellow is detected
        if max(b_mean, g_mean, r_mean) == r_mean:
            print("Dominant color: Red")
            red_flag = True  # Assign True to red_flag
            red_color = 1
        elif max(b_mean, g_mean, r_mean) == g_mean:
            print("Dominant color: Green")
            green_flag = True  # Assign True to green_flag
            green_color = 1
        elif max(b_mean, g_mean, r_mean) == b_mean:
            print("Dominant color: Blue")
            blue_flag = True  # Assign True to blue_flag
            blue_color = 1

def detect_yellow(frame):
    global yellow_flag, yellow_color

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range of yellow color in HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask=mask)

    # Check if yellow is detected
    if np.any(res):
        print("Yellow detected")
        yellow_flag = True  # Set yellow_flag to True
        yellow_color = 1


while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame")
        break

    # Resize the frame to make processing faster (optional)
    frame = cv2.resize(frame, (640, 480))

    # Check flags and process accordingly
    if not (red_flag or blue_flag or yellow_flag):
        # Find and print the dominant color
        find_dominant_color(frame)

    # Detect yellow
    if not yellow_flag:
        detect_yellow(frame)

    # Display the resulting frame (optional)
    cv2.imshow('Frame', frame)

    # Check flags and print messages
    if red_flag or blue_flag or green_flag:
        print("Team color updated")
    if yellow_flag:
        print("Zone 3 found")
        break  # Exit the loop once yellow is detected

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture and close windows
cap.release()
cv2.destroyAllWindows()
