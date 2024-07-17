import cv2
import numpy as np
import RPi.GPIO as GPIO
from time import sleep
import threading
import smbus  # Communication I2C
import math
from gpiozero import Button
from signal import pause

# Setup GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

AN1 = 24  # Locomotion
AN2 = 25
AN3 = 17
AN4 = 27
AN5 = 11  # Conveyor motor
AN6 = 8  # Gripper motor

DIG1 = 18  # Locomotion
DIG2 = 23
DIG3 = 22
DIG4 = 5
DIG5 = 15  # Conveyor motor
DIG6 = 7  # Gripper motor

limitswitch_1 = 29  # Conveyor upper limit switch
limitswitch_2 = 13  # Conveyor lower limit switch
limitswitch_3 = 16  # Gripper limit switch
limitswitch_4 = 12  # Gripper limit switch

GPIO.setup([AN1, AN2, AN3, AN4, AN5, AN6, DIG1, DIG2, DIG3, DIG4, DIG5, DIG6], GPIO.OUT)
GPIO.setup([limitswitch_1, limitswitch_2, limitswitch_3, limitswitch_4], GPIO.IN, pull_up_down=GPIO.PUD_UP)

sleep(1)

p1 = GPIO.PWM(AN1, 255)
p2 = GPIO.PWM(AN2, 255)
p3 = GPIO.PWM(AN3, 255)
p4 = GPIO.PWM(AN4, 255)
p5 = GPIO.PWM(AN5, 255)
p6 = GPIO.PWM(AN6, 255)

# Functions to control the robot's movement
def movefrontdir():
    print("Move Forward")
    GPIO.output([DIG1, DIG4], GPIO.HIGH)
    GPIO.output([DIG2, DIG3], GPIO.LOW)
    p1.start(35)
    p2.start(35)
    p3.start(35)
    p4.start(35)

def movebackdir():
    print("Move Backward")
    GPIO.output([DIG1, DIG4], GPIO.LOW)
    GPIO.output([DIG2, DIG3], GPIO.HIGH)
    p1.start(35)
    p2.start(35)
    p3.start(35)
    p4.start(35)

def moveleftdir():
    print("Move Left")
    GPIO.output([DIG1, DIG2], GPIO.LOW)
    GPIO.output([DIG3, DIG4], GPIO.HIGH)
    p1.start(35)
    p2.start(35)
    p3.start(35)
    p4.start(35)

def moverightdir():
    print("Move Right")
    GPIO.output([DIG1, DIG2], GPIO.HIGH)
    GPIO.output([DIG3, DIG4], GPIO.LOW)
    p1.start(35)
    p2.start(35)
    p3.start(35)
    p4.start(35)

def stop():
    print("Stop")
    GPIO.output([DIG1, DIG4], GPIO.HIGH)
    GPIO.output([DIG2, DIG3], GPIO.LOW)
    p1.start(0)
    p2.start(0)
    p3.start(0)
    p4.start(0)

# Initialize camera and detect team color
webcam = cv2.VideoCapture(0)

def find_dominant_color(frame):
    b, g, r = cv2.split(frame)
    b_mean = np.mean(b)
    g_mean = np.mean(g)
    r_mean = np.mean(r)
    
    if max(b_mean, g_mean, r_mean) == r_mean:
        return "red"
    elif max(b_mean, g_mean, r_mean) == b_mean:
        return "blue"
    elif (g_mean >= 150 and r_mean >= 150) or (r_mean >= 200 and g_mean >= 100):
        return "yellow"
    else:
        return "unknown"

while True:
    ret, imageFrame = webcam.read()
    if not ret:
        continue

    team_color = find_dominant_color(imageFrame)
    
    if team_color in ["red", "blue"]:
        break

    cv2.imshow('Frame', imageFrame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

sleep(2)

# Line follower function
def zeroloop():
    video_capture = cv2.VideoCapture(0)
    video_capture.set(3, 50)
    video_capture.set(4, 60)

    while True:
        ret, frame = video_capture.read()
        if not ret:
            continue

        crop_img = frame[10:300, 10:300]
        hsv_frame = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        thresh = cv2.inRange(hsv_frame, lower_white, upper_white)
        contours, _ = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
                cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)
                cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)
                
                if cx >= 186:
                    moverightdir()
                elif 124 < cx < 186:
                    movefrontdir()
                elif 0 < cx < 124:
                    moveleftdir()
            else:
                zeroloop()
        else:
            stop()
        
        cv2.imshow('Frame', crop_img)
        cv2.imshow('HSV Frame', hsv_frame)

        if yellow_zone == 1:
            sleep(7)
            break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

zeroloop()

# Ball detection and movement adjustment
def ball_detection(team_color):
    if team_color == "red":
        color_lower = np.array([0, 125, 150], np.uint8)
        color_upper = np.array([255, 255, 255], np.uint8)
    else:
        color_lower = np.array([100, 150, 0], np.uint8)  # Example values for blue
        color_upper = np.array([140, 255, 255], np.uint8)

    while True:
        ret, imageFrame = webcam.read()
        if not ret:
            continue

        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(hsvFrame, color_lower, color_upper)
        kernel = np.ones((5, 5), np.uint8)
        color_mask = cv2.dilate(color_mask, kernel)
        contours, _ = cv2.findContours(color_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 2000:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)
                z_distance = (195 * 1000) / (radius * 2)
                X_distance = center[0] - imageFrame.shape[1] // 2
                Y_distance = center[1] - imageFrame.shape[0] // 2

                if abs(X_distance) > 40:
                    if X_distance > 0:
                        moveleftdir()
                    else:
                        moverightdir()
                elif abs(z_distance) > 200:
                    if z_distance > 200:
                        movebackdir()
                    else:
                        movefrontdir()
                else:
                    stop()

                if GPIO.input(limitswitch_3) == 0 and GPIO.input(limitswitch_4) == 0:
                    GPIO.output(DIG6, GPIO.LOW)
                    p6.start(75)
                else:
                    GPIO.output(DIG6, GPIO.LOW)
                    p6.start(0)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        cv2.imshow('Ball Detection', imageFrame)
        cv2.imshow('HSV Frame', hsvFrame)

ball_detection(team_color)

# Release resources
webcam.release()
cv2.destroyAllWindows()
GPIO.output([DIG2, DIG3, DIG4], GPIO.LOW)
p1.stop()
p2.stop()
p3.stop()
p4.stop()