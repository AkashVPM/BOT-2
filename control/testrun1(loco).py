import cv2
import numpy as np
import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode (GPIO.BCM)
GPIO.setwarnings (False)
AN1 = 24
AN2 = 25
AN3 = 17
AN4 = 27

DIG1 = 18
DIG2 = 23
DIG3 = 22
DIG4 = 5

GPIO.setup(AN1, GPIO.OUT)
GPIO.setup(AN2, GPIO.OUT)
GPIO.setup(AN3, GPIO.OUT)
GPIO.setup(AN4, GPIO.OUT)
GPIO.setup(DIG1, GPIO.OUT)
GPIO.setup(DIG2, GPIO.OUT)
GPIO.setup(DIG3, GPIO.OUT)
GPIO.setup(DIG4, GPIO.OUT)
sleep (1)

p1 = GPIO.PWM(AN1, 255)
p2 = GPIO.PWM(AN2, 255)
p3 = GPIO.PWM(AN3, 255)
p4 = GPIO.PWM(AN4, 255)

def movefrontdir():
    print("front")
    GPIO.output(DIG1, GPIO.HIGH)
    GPIO.output(DIG2, GPIO.LOW)
    GPIO.output(DIG3, GPIO.LOW)
    GPIO.output(DIG4, GPIO.HIGH)
    p1.start(35)
    p2.start(35)
    p3.start(35)
    p4.start(35)

def movebackdir():
    print("back")
    GPIO.output(DIG1, GPIO.LOW)
    GPIO.output(DIG2, GPIO.HIGH)
    GPIO.output(DIG3, GPIO.HIGH)
    GPIO.output(DIG4, GPIO.LOW)
    p1.start(35)
    p2.start(35)
    p3.start(35)
    p4.start(35)


def moveleftdir():
    print("left")
    GPIO.output(DIG1, GPIO.LOW)
    GPIO.output(DIG2, GPIO.LOW)
    GPIO.output(DIG3, GPIO.HIGH)
    GPIO.output(DIG4, GPIO.HIGH)
    p1.start(35)
    p2.start(35)
    p3.start(35)
    p4.start(35)

def moverightdir():
    print("right")
    GPIO.output(DIG1, GPIO.HIGH)
    GPIO.output(DIG2, GPIO.HIGH)
    GPIO.output(DIG3, GPIO.LOW)
    GPIO.output(DIG4, GPIO.LOW)
    p1.start(35)
    p2.start(35)
    p3.start(35)
    p4.start(35)

def stop():
    print("stop")
    GPIO.output(DIG1, GPIO.HIGH)
    GPIO.output(DIG2, GPIO.HIGH)
    GPIO.output(DIG3, GPIO.LOW)
    GPIO.output(DIG4, GPIO.LOW)
    p1.start(0)
    p2.start(0)
    p3.start(0)
    p4.start(0)

def zeroloop():
    video_capture = cv2.VideoCapture(0)
    video_capture.set(3, 50)
    video_capture.set(4,60)

    while(True):
       ret, frame = video_capture.read()
       crop_img = frame[10:300, 10:300]

      #gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
      #blur = cv2.GaussianBlur(gray, (5, 5), 0)
      #ret, thresh = cv2.threshold(blur,200, 255,cv2.THRESH_BINARY)
      # Convert frame to HSV color space

       hsv_frame = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        # Define range of white color in HSV (adjust as needed)
       lower_white = np.array([0, 0, 200])
       upper_white = np.array([ 180, 30, 255])

        # Threshold the HSV image to get only white colors
       thresh=cv2.inRange(hsv_frame, lower_white, upper_white)
       contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)


       if len(contours) > 0:
           c = max(contours, key=cv2.contourArea)
           M = cv2.moments(c)

           if M['m00']!= 0:
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
                elif 0< cx < 124:
                    print("Move Left!")
                    moveleftdir()
           else:
                print("zero coming da")
                zeroloop()
       else:
           print("I don't see the line")
           stop()

       cv2.imshow('frame', crop_img)
       cv2.imshow('frame1',hsv_frame)
       if cv2.waitKey(1) & 0xFF == ord('q'):
           break

zeroloop()