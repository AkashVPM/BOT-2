import cv2
import numpy as np
import threading
# import smbus  # comunicación I2C
import math

# Initialize webcam
webcam = cv2.VideoCapture(0)

# Define color ranges for detection
lower_white = np.array([0, 0, 200])
upper_white = np.array([255, 255, 255])
lower_red = np.array([0, 125, 150])
upper_red = np.array([255, 255, 255])
lower_blue = np.array([45.86, 183.09, 182.58])
upper_blue = np.array([255, 255, 255])
yellow_lower = np.array([22, 93, 0])
yellow_upper = np.array([45, 255, 255])

# Color detection initial setup
Team_Red, Team_Blue, yellow_zone = 0, 0, 0

def find_dominant_color(frame):
    global Team_Red, Team_Blue, yellow_zone
    b, g, r = cv2.split(frame)
    b_mean, g_mean, r_mean = np.mean(b), np.mean(g), np.mean(r)
    if max(b_mean, g_mean, r_mean) == r_mean:
        Team_Red, Team_Blue = 1, 0
    elif max(b_mean, g_mean, r_mean) == g_mean:
        Team_Blue, Team_Red = 1, 0
    elif (g_mean >= 150 and r_mean >= 150) or (r_mean >= 200 and g_mean >= 100):
        yellow_zone = 1
    else:
        Team_Blue, Team_Red = 1, 0

while True:
    _, imageFrame = webcam.read()
    find_dominant_color(imageFrame)
    if Team_Red == 1 or Team_Blue == 1:
        break

# Line following function
def zeroloop():
    video_capture = cv2.VideoCapture(0)
    video_capture.set(3, 50)
    video_capture.set(4, 60)

    while True:
        ret, frame = video_capture.read()
        crop_img = frame[10:300, 10:300]
        hsv_frame = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv_frame, lower_white, upper_white)
        contours, _ = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)

            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
                cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)
                cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)
                print(cx)
                if cx >= 186:
                    print("Turn Right")
                elif 124 < cx < 186:
                    print("On Track!")
                elif 0 < cx < 124:
                    print("Move Left!")
            else:
                print("zero coming da")
                zeroloop()
        else:
            print("I don't see the line")

        cv2.imshow('frame', crop_img)
        cv2.imshow('frame1', hsv_frame)
        if yellow_zone == 1:
            threading.Event().wait(7)
            break

zeroloop()

# Ball detection and picking function
def ball_detection(team_color):
    while True:
        _, imageFrame = webcam.read()
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
        rows, cols = imageFrame.shape[:2]
        x1, y1 = rows / 2, cols / 2
        color_lower = lower_red if team_color == "red" else lower_blue
        color_upper = upper_red if team_color == "red" else upper_blue
        color_mask = cv2.inRange(hsvFrame, color_lower, color_upper)
        kernel = np.ones((5, 5), np.uint8)
        color_mask = cv2.dilate(color_mask, kernel)
        contours, _ = cv2.findContours(color_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        X_distance = None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 2000:
                (x2, y2), radius = cv2.minEnclosingCircle(contour)
                center = (int(x2), int(y2))
                radius = int(radius)
                cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)
                cv2.putText(imageFrame, f"{team_color.capitalize()} Ball", center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255))
                break

        cv2.imshow("Multiple Color Detection", imageFrame)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            webcam.release()
            cv2.destroyAllWindows()
            break

# Color of the team ball to detect
team_color = "red" if Team_Red else "blue"
ball_detection(team_color)
