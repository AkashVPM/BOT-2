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


Team_Red, Team_Blue, yellow_zone = 0, 0, 0
red_set, blue_set, yellow_set = False, False, False

def find_dominant_color(frame):
    global Team_Red, Team_Blue, yellow_zone, red_set, blue_set, yellow_set
    
    b, g, r = cv2.split(frame)
    b_mean, g_mean, r_mean = np.mean(b), np.mean(g), np.mean(r)
    
    if not red_set and not blue_set :  # Only update if none of the flags are set
        if max(b_mean, g_mean, r_mean) == r_mean:
            Team_Red, Team_Blue = 1, 0
            red_set = True
            print("red find")
        elif max(b_mean, g_mean, r_mean) == g_mean:
            Team_Blue, Team_Red = 1, 0
            blue_set = True
            print("blue find")
        else:
            Team_Blue, Team_Red = 1, 0
    
    # Yellow zone update
    if not yellow_set:
        if (g_mean >= 150 and r_mean >= 150) or (r_mean >= 200 and g_mean >= 100):
            yellow_zone = 1
            yellow_set = True
            print("yellow find")
        else:
            yellow_zone = 0

