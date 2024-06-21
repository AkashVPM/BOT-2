
# Write your code here :-)
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
DIG5 = 12
DIG6 = 16
DIG7 = 20
DIG8 = 21


GPIO.setup(AN1, GPIO.OUT)
GPIO.setup(AN2, GPIO.OUT)
GPIO.setup(AN3, GPIO.OUT)
GPIO.setup(AN4, GPIO.OUT)
GPIO.setup(DIG1, GPIO.OUT)
GPIO.setup(DIG2, GPIO.OUT)
GPIO.setup(DIG3, GPIO.OUT)
GPIO.setup(DIG4, GPIO.OUT)
GPIO.setup(DIG5, GPIO.OUT)
GPIO.setup(DIG6, GPIO.OUT)
GPIO.setup(DIG7, GPIO.OUT)
GPIO.setup(DIG8, GPIO.OUT)

sleep (1)

p1 = GPIO.PWM(AN1, 255)
p2 = GPIO.PWM(AN2, 255)
p3 = GPIO.PWM(AN3, 255)
p4 = GPIO.PWM(AN4, 255)

try:
   while True:
       print("Forward")
       GPIO.output(DIG1, GPIO.HIGH)
       GPIO.output(DIG2, GPIO.LOW)
       GPIO.output(DIG3, GPIO.LOW)
       GPIO.output(DIG4, GPIO.HIGH)
       p1.start(50)
       p2.start(50)
       p3.start(50)
       p4.start(50)
       sleep (10)

       print("Backward")
       GPIO.output(DIG1, GPIO.LOW)
       GPIO.output(DIG2, GPIO.HIGH)
       GPIO.output(DIG3, GPIO.HIGH)
       GPIO.output(DIG4, GPIO.LOW)
       p1.start(100)
       p2.start(100)
       p3.start(100)
       p4.start(100)
       sleep(10)

       print("Left")
       GPIO.output(DIG1, GPIO.LOW)
       GPIO.output(DIG2, GPIO.LOW)
       GPIO.output(DIG3, GPIO.HIGH)
       GPIO.output(DIG4, GPIO.HIGH)
       p1.start(50)
       p2.start(50)
       p3.start(50)
       p4.start(50)
       sleep (10)

       print("Right")
       GPIO.output(DIG1, GPIO.HIGH)
       GPIO.output(DIG2, GPIO.HIGH)
       GPIO.output(DIG3, GPIO.LOW)
       GPIO.output(DIG4, GPIO.LOW)
       p1.start(100)
       p2.start(100)
       p3.start(100)
       p4.start(100)
       sleep(5)

       print("STEPPER_UP")
       GPIO.output(DIG5, GPIO.HIGH)
       GPIO.output(DIG6, GPIO.LOW)
       sleep(10)

       print("STEPPER_DOWN")
       GPIO.output(DIG5, GPIO.LOW)
       GPIO.output(DIG6, GPIO.HIGH)
       sleep(10)

       print("GRIPPER_UP")
       GPIO.output(DIG7, GPIO.HIGH)
       GPIO.output(DIG8, GPIO.LOW)

       print("GRIPPER_DOWN")
       GPIO.output(DIG8, GPIO.HIGH)
       GPIO.output(DIG7, GPIO.LOW)

except:
        p1.start(0)
        p2.start(0)
        p3.start(0)
        p4.start(0)