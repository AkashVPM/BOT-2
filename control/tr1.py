import cv2
import numpy as np
import RPi.GPIO as GPIO
from time import sleep
import threading
import smbus  # comunicación I2C
import math

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin definitions
AN1, AN2, AN3, AN4, AN5, AN6 = 24, 25, 17, 27, 11, 8
DIG1, DIG2, DIG3, DIG4, DIG5, DIG6 = 18, 23, 22, 5, 15, 7
limitswitch_1, limitswitch_2, limitswitch_3, limitswitch_4 = 29, 13, 16, 12

# GPIO pin setup
pins = [AN1, AN2, AN3, AN4, AN5, AN6, DIG1, DIG2, DIG3, DIG4, DIG5, DIG6]
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)
limit_switches = [limitswitch_1, limitswitch_2, limitswitch_3, limitswitch_4]
for switch in limit_switches:
    GPIO.setup(switch, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# PWM setup
p1, p2, p3, p4, p5, p6 = [GPIO.PWM(pin, 255) for pin in [AN1, AN2, AN3, AN4, AN5, AN6]]

# Movement functions
def movefrontdir():
    print("front")
    GPIO.output(DIG1, GPIO.HIGH)
    GPIO.output(DIG2, GPIO.LOW)
    GPIO.output(DIG3, GPIO.LOW)
    GPIO.output(DIG4, GPIO.HIGH)
    for pwm in [p1, p2, p3, p4]:
        pwm.start(100)

def movebackdir():
    print("back")
    GPIO.output(DIG1, GPIO.LOW)
    GPIO.output(DIG2, GPIO.HIGH)
    GPIO.output(DIG3, GPIO.HIGH)
    GPIO.output(DIG4, GPIO.LOW)
    for pwm in [p1, p2, p3, p4]:
        pwm.start(100)

def moveleftdir():
    print("left")
    GPIO.output(DIG1, GPIO.LOW)
    GPIO.output(DIG2, GPIO.LOW)
    GPIO.output(DIG3, GPIO.HIGH)
    GPIO.output(DIG4, GPIO.HIGH)
    for pwm in [p1, p2, p3, p4]:
        pwm.start(100)

def moverightdir():
    print("right")
    GPIO.output(DIG1, GPIO.HIGH)
    GPIO.output(DIG2, GPIO.HIGH)
    GPIO.output(DIG3, GPIO.LOW)
    GPIO.output(DIG4, GPIO.LOW)
    for pwm in [p1, p2, p3, p4]:
        pwm.start(100)

def stop():
    print("stop")
    GPIO.output(DIG1, GPIO.HIGH)
    GPIO.output(DIG2, GPIO.LOW)
    GPIO.output(DIG3, GPIO.LOW)
    GPIO.output(DIG4, GPIO.HIGH)
    for pwm in [p1, p2, p3, p4]:
        pwm.start(0)

real_ball_diameter = 195  # Diameter of the ball in millimeters
focal_length = 1000       # Focal length of the camera in pixels (example value)

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

# Initialize webcam
webcam = cv2.VideoCapture(0)
while True:
    _, imageFrame = webcam.read()
    find_dominant_color(imageFrame)
    if Team_Red == 1 or Team_Blue == 1:
        break

sleep(2)

# Line following function
def zeroloop():
    video_capture = cv2.VideoCapture(0)
    video_capture.set(3, 50)
    video_capture.set(4, 60)

    while True:
        ret, frame = video_capture.read()
        if not ret:
            print("Failed to grab frame")
            continue

        crop_img = frame[10:300, 10:300]

        # Convert frame to HSV color space
        hsv_frame = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        # Define range of white color in HSV (adjust as needed)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])

        # Threshold the HSV image to get only white colors
        thresh = cv2.inRange(hsv_frame, lower_white, upper_white)
        contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

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
                    moverightdir()
                elif 124 < cx < 186:
                    print("On Track!")
                    movefrontdir()
                elif 0 < cx < 124:
                    print("Move Left!")
                    moveleftdir()
            else:
                print("zero coming da")
                zeroloop()
        else:
            print("I don't see the line")
            stop()

        cv2.imshow('frame', crop_img)
        cv2.imshow('frame1', hsv_frame)
        # Exit after 5 sec of yellow detection
        if yellow_zone == 1:
            start_time = threading.Event()
            start_time.wait(7)
            break  # Need to change the condition

# Initialize and start the line following loop
zeroloop()
# Ball detection and picking function
def ball_detection(team_color):
    while True:
        _, imageFrame = webcam.read()
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
        rows, cols = imageFrame.shape[:2]
        x1, y1 = rows / 2, cols / 2
        color_lower = np.array([0, 125, 150], np.uint8) if team_color == "red" else np.array([45.86, 183.09, 182.58], np.uint8)
        color_upper = np.array([255, 255, 255], np.uint8)
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
                apparent_diameter = radius * 2
                z_distance = (real_ball_diameter * focal_length) / apparent_diameter
                cv2.putText(imageFrame, f"Z_Distance: {z_distance:.2f} Millimeters", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                X_distance = x2 - x1
                Y_distance = y2 - y1
                cv2.putText(imageFrame, f"X_Distance: {X_distance:.2f} Millimeters", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 79, 255), 2)
                cv2.putText(imageFrame, f"Y_Distance: {Y_distance:.2f} Millimeters", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 79, 255), 2)
                break

        cv2.imshow("Multiple Color Detection", imageFrame)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            webcam.release()
            cv2.destroyAllWindows()
            break
        elif X_distance is not None and -5 < X_distance < 5:
            stop()
            break
        elif X_distance is not None:
            if X_distance > 5:
                moverightdir()
            elif X_distance < -5:
                moveleftdir()

# Color of the team ball to detect
team_color = "red" if Team_Red else "blue"
ball_detection(team_color)

# Picking mechanism
def ball_picking():
    movebackdir()
    while GPIO.input(limitswitch_1) == GPIO.HIGH:
        pass
    stop()

    moverightdir()
    while GPIO.input(limitswitch_3) == GPIO.HIGH:
        pass
    stop()

    movefrontdir()
    sleep(1)
    stop()

    moveleftdir()
    while GPIO.input(limitswitch_4) == GPIO.HIGH:
        pass
    stop()

    movebackdir()
    while GPIO.input(limitswitch_1) == GPIO.HIGH:
        pass
    stop()

ball_picking()

# Delivering the ball to the basket
def ball_delivery():
    movefrontdir()
    while GPIO.input(limitswitch_2) == GPIO.HIGH:
        pass
    stop()

    moveleftdir()
    while GPIO.input(limitswitch_3) == GPIO.HIGH:
        pass
    stop()

    movebackdir()
    while GPIO.input(limitswitch_1) == GPIO.HIGH:
        pass
    stop()

    moveleftdir()
    while GPIO.input(limitswitch_4) == GPIO.HIGH:
        pass
    stop()

ball_delivery()
