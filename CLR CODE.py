import numpy as np
import cv2

# Define known parameters
real_ball_diameter = 195  # Diameter of the ball in meters (10 cm)
focal_length = 1000       # Focal length of the camera in pixels (example value)

# Capturing video through webcam
webcam = cv2.VideoCapture(0)

while True:
    # Reading the video from the webcam in image frames
    _, imageFrame = webcam.read()

    # Convert the imageFrame from BGR (RGB color space) to HSV (hue-saturation-value) color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    # Set range for yellow color and define mask
    rows, cols = imageFrame.shape[:2]
import numpy as np
import cv2

# Define known parameters
real_ball_diameter = 195  # Diameter of the ball in meters (10 cm)
focal_length = 1000       # Focal length of the camera in pixels (example value)

# Capturing video through webcam
webcam = cv2.VideoCapture(0)

while True:
    # Reading the video from the webcam in image frames
    _, imageFrame = webcam.read()

    # Convert the imageFrame from BGR (RGB color space) to HSV (hue-saturation-value) color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    # Set range for yellow color and define mask
    rows, cols = imageFrame.shape[:2]
    x1 = rows/2
    y1 = cols/2
    yellow_lower = np.array([0, 100, 100], np.uint8)
    yellow_upper = np.array([0, 255, 255], np.uint8)
    yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)

    # Morphological Transform, Dilation for yellow color
    kernel = np.ones((5, 5), np.uint8)
    yellow_mask = cv2.dilate(yellow_mask, kernel)

    # Find contours
    contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw circles around contours and calculate distance
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 2000:
            # Calculate the radius of the enclosing circle
            (x2, y2), radius = cv2.minEnclosingCircle(contour)
            center = (int(x2), int(y2))
            radius = int(radius)
            cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)
            cv2.putText(imageFrame, "Red Ball", center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255))

            # Calculate distance using the simplified formula
            apparent_diameter = radius * 2  # Apparent diameter of the ball in pixels
            z_distance = (real_ball_diameter * focal_length) / apparent_diameter
            cv2.putText(imageFrame, f"Distance: {z_distance:.2f} Millimeters", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            y_distance = x2-x1
            x_distance = y2-y1
            cv2.putText(imageFrame, f"Distance: {y_distance:.2f} Millimeters", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 79, 255), 2)
            cv2.putText(imageFrame, f"Distance: {x_distance:.2f} Millimeters", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 100, 255), 2)
            # Displaying the output
    cv2.imshow("Red Ball Detection in Real-Time", imageFrame)

    # Exiting the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
webcam.release()
cv2.destroyAllWindows()
    x1 = rows/2
    y1 = cols/2
    yellow_lower = np.array([0, 100, 100], np.uint8)
    yellow_upper = np.array([0, 255, 255], np.uint8)
    yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)

    # Morphological Transform, Dilation for yellow color
    kernel = np.ones((5, 5), np.uint8)
    yellow_mask = cv2.dilate(yellow_mask, kernel)

    # Find contours
    contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw circles around contours and calculate distance
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 2000:
            # Calculate the radius of the enclosing circle
            (x2, y2import numpy as np

import cv2
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

# Define known parameters
real_ball_diameter = 195  # Diameter of the ball in meters (10 cm)
focal_length = 1000       # Focal length of the camera in pixels (example value)

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
    sleep(1)

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


# Capturing video through webcam
webcam = cv2.VideoCapture(0)

while True:
    # Reading the video from the webcam in image frames
    _, imageFrame = webcam.read()

    # Convert the imageFrame from BGR (RGB color space) to HSV (hue-saturation-value) color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    # Set range for yellow color and define mask
    rows, cols = imageFrame.shape[:2]
    x1 = rows/2
    y1 = cols/2
    center1 = (int(x1), int(y1))
    radius1 = 10
    cv2.circle(imageFrame, center1, radius1, (0, 255, 255), 2)
    red_lower = np.array([0, 125 , 150], np.uint8)
    red_upper = np.array([255, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Morphological Transform, Dilation for yellow color
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.dilate(red_mask, kernel)

    # Find contours
    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    X_distance = None
    # Draw circles around contours and calculate distance
    for contour in contours:
        area = cv2.import numpy as np

import cv2
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

# Define known parameters
real_ball_diameter = 195  # Diameter of the ball in meters (10 cm)
focal_length = 1000       # Focal length of the camera in pixels (example value)

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
    sleep(1)

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


# Capturing video through webcam
webcam = cv2.VideoCapture(0)

while True:
    # Reading the video from the webcam in image frames
    _, imageFrame = webcam.read()

    # Convert the imageFrame from BGR (RGB color space) to HSV (hue-saturation-value) color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    # Set range for yellow color and define mask
    rows, cols = imageFrame.shape[:2]
    x1 = rows/2
    y1 = cols/2
    center1 = (int(x1), int(y1))
    radius1 = 10
    cv2.circle(imageFrame, center1, radius1, (0, 255, 255), 2)
    red_lower = np.array([0, 125 , 150], np.uint8)
    red_upper = np.import numpy as np

import cv2
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

# Define known parameters
real_ball_diameter = 195  # Diameter of the ball in meters (10 cm)
focal_length = 1000       # Focal length of the camera in pixels (example value)

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
    sleep(1)

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


# Capturing video through webcam
webcam = cv2.VideoCapture(0)

while True:
    # Reading the video from the webcam in image frames
    _, imageFrame = webcam.read()

    # Convert the imageFrame from BGR (RGB color space) to HSV (hue-saturation-value) color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    # Set range for yellow color and define mask
    rows, cols = imageFrame.shape[:2]
    x1 = rows/2
    y1 = cols/2
    center1 = (int(x1), int(y1))
    radius1 = 10
    cv2.circle(imageFrame, center1, radius1, (0, 255, 255), 2)
    red_lower = np.array([0, 125 , 150], np.uint8)
    red_upper = np.array([255, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Morphological Transform, Dilation for yellow color
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.dilate(red_mask, kernel)

    # Find contours
    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    X_distance = None
    # Draw circles around contours and calculate distance
    for contour in contours:
        area = cv2.import numpy as np

import cv2
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

# Define known parameters
real_ball_diameter = 195  # Diameter of the ball in meters (10 cm)
focal_length = 1000       # Focal length of the camera in pixels (example value)

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
    sleep(1)

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


# Capturing video through webcam
webcam = cv2.VideoCapture(0)

while True:
    # Reading the video from the webcam in image frames
    _, imageFrame = webcam.read()

    # Convert the imageFrame from BGR (RGB color space) to HSV (hue-saturation-value) color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    # Set range for yellow color and define mask
    rows, cols = imageFrame.shape[:2]
    x1 = rows/2
    y1 = cols/2
    center1 = (int(x1), int(y1))
    radius1 = 10
    cv2.circle(imageFrame, center1, radius1, (0, 255, 255), 2)
    red_lower = np.array([0, 125 , 150], np.uint8)
    red_upper = np.array([255, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Morphological Transform, Dilation for yellow color
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.dilate(red_mask, kernel)

    # Find contours
    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    X_distance = None
    # Draw circles around contours and calculate distance
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 2000:
            # Calculate the radius of the enclosing circle
            (x2, y2), radius = cv2.minEnclosingCircle(contour)
            center = (int(x2), int(y2))
            radius = int(radius)

            cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)
            cv2.putText(imageFrame, "Red Ball", center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255))
import numpy as np

import cv2
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

# Define known parameters
real_ball_diameter = 195  # Diameter of the ball in meters (10 cm)
focal_length = 1000       # Focal length of the camera in pixels (example value)

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
    sleep(1)

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


# Capturing video through webcam
webcam = cv2.VideoCapture(0)

while True:
    # Reading the video from the webcam in image frames
    _, imageFrame = webcam.read()

    # Convert the imageFrame from BGR (RGB color space) to HSV (hue-saturation-value) color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    # Set range for yellow color and define mask
    rows, cols = imageFrame.shape[:2]
    x1 = rows/2
    y1 = cols/2
    center1 = (int(x1), int(y1))
    radius1 = 10
    cv2.circle(imageFrame, center1, radius1, (0, 255, 255), 2)
    red_lower = np.array([0, 125 , 150], np.uint8)
    red_upper = np.array([255, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Morphological Transform, Dilation for yellow color
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.dilate(red_mask, kernel)

    # Find contours
    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    X_distance = None
    # Draw circles around contours and calculate distance
    for contour in contours:
        area = cv2.import numpy as np

import cv2
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

# Define known parameters
real_ball_diameter = 195  # Diameter of the ball in meters (10 cm)
focal_length = 1000       # Focal length of the camera in pixels (example value)

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
    sleep(1)

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


# Capturing video through webcam
webcam = cv2.VideoCapture(0)

while True:
    # Reading the video from the webcam in image frames
    _, imageFrame = webcam.read()

    # Convert the imageFrame from BGR (RGB color space) to HSV (hue-saturation-value) color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    # Set range for yellow color and define mask
    rows, cols = imageFrame.shape[:2]
    x1 = rows/2
    y1 = cols/2
    center1 = (int(x1), int(y1))
    radius1 = 10
    cv2.circle(imageFrame, center1, radius1, (0, 255, 255), 2)
    red_lower = np.array([0, 125 , 150], np.uint8)
    red_upper = np.array([255, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Morphologicalimport numpy as np

import cv2
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

# Define known parameters
real_ball_diameter = 195  # Diameter of the ball in meters (10 cm)
focal_length = 1000       # Focal length of the camera in pixels (example value)

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
    sleep(1)

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


# Capturing video through webcam
webcam = cv2.VideoCapture(0)

while True:
    # Reading the video from the webcam in image frames
    _, imageFrame = webcam.read()

    # Convert the imageFrame from BGR (RGB color space) to HSV (hue-saturation-value) color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    # Set range for yellow color and define mask
    rows, cols = imageFrame.shape[:2]
    x1 = rows/2
    y1 = cols/2
    center1 = (int(x1), int(y1))
    radius1 = 10
    cv2.circle(imageFrame, center1, radius1, (0, 255, 255), 2)
    red_lower = np.array([0, 125 , 150], np.uint8)
    red_upper = np.array([255, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Morphological Transform, Dilation for yellow color
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.dilate(red_mask, kernel)

    # Find contours
    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    X_distance = None
    # Draw circles around contours and calculate distance
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 2000:
            # Calculate the radius of the enclosing circle
            (x2, y2), radius = cv2.minEnclosingCircle(contour)
            center = (int(x2), int(y2))
            radius = int(radius)

            cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)
            cv2.putText(imageFrame, "Red Ball", center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255))

            # Calculate distance using the simplified formula
            apparent_diameter = radius * 2  # Apparent diameter of the ball in pixels
            z_distaimport numpy as np

import cv2
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

# Define known parameters
real_ball_diameter = 195  # Diameter of the ball in meters (10 cm)
focal_length = 1000       # Focal length of the camera in pixels (example value)

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
    sleep(1)

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
    GPIO.output(DIGimport numpy as np

import cv2
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

# Define known parameters
real_ball_diameter = 195  # Diameter of the ball in meters (10 cm)
focal_length = 1000       # Focal length of the camera in pixels (example value)

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
    sleep(1)

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


# Capturing video through webcam
webcam = cv2.VideoCapture(0)

while True:
    # Reading the video from the webcam in image frames
    _, imageFrame = webcam.read()

    # Convert the imageFrame from BGR (RGB color space) to HSV (hue-saturation-value) color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    # Set range for yellow color and define mask
    rows, cols = imageFrame.shape[:2]
    x1 = rows/2
    y1 = cols/2
    center1 = (int(x1), int(y1))
    radius1 = 10
    cv2.circle(imageFrame, center1, radius1, (0, 255, 255), 2)
    red_lower = np.array([0, 125 , 150], np.uint8)
    red_upper = np.array([255, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Morphological Transform, Dilation for yellow color
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.dilate(red_mask, kernel)

    # Find contours
    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    X_distance = None
    # Draw circles around contours and calculate distance
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 2000:
            # Calculate the radius of the enclosing circle
            (x2, y2), radius = cv2.minEnclosingCircle(contour)
            center = (int(x2), int(y2))
            radius = int(radius)

            cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)
            cv2.putText(imageFrame, "Red Ball", center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255))

            # Calculate distance using the simplified formula
            apparent_diameter = radius * 2  # Apparent diameter of the ball in pixels
            z_distance = (real_ball_diameter * focal_length) / apparent_diameter
            cv2.putText(imageFrame, f"Z_Distance: {z_distance:.2f} Millimeters", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            X_distance = x2-x1
            print(X_distance)
            Y_distance = y2-y1
            cv2.putText(imageFrame, f"X_Distance: {X_distance:.2f} Millimeters", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 79, 255), 2)
            cv2.putText(imageFrame, f"Y_Distance: {Y_distance:.2f} Millimeters", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 100, 255), 2)
            if X_distance != 0:
                if X_distance > 0 :
                    moveleftdir()
                elif X_distance < 0:
                   moverightdir()
            elif X_distance < 40 and X_distance > -80 and z_distance != 0:
                if z_distance > 0:
                    movebackdir()
                if z_distance <= 0:
                    movefrontdir()
            elif z_distance == 0 and X_distance == 0:
                stop()

    if X_distance == None:
        print('No x2')
        stop()

       # switch on the gripper motor
            # Displaying the output
    cv2.imshow("Red Ball Detection in Real-Time", imageFrame)
    cv2.imshow("Red Ball Detection in Real-Time_hsv", hsvFrame)

    # Exiting the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
webcam.release()
cv2.destroyAllWindows()1, GPIO.HIGH)
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


# Capturing video through webcam
webcam = cv2.VideoCapture(0)

while True:
    # Reading the video from the webcam in image frames
    _, imageFrame = webcam.read()

    # Convert the imageFrame from BGR (RGB color space) to HSV (hue-saturation-value) color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    # Set range for yellow color and define mask
    rows, cols = imageFrame.shape[:2]
    x1 = rows/2
    y1 = cols/2
    center1 = (int(x1), int(y1))
    radius1 = 10
    cv2.circle(imageFrame, center1, radius1, (0, 255, 255), 2)
    red_lower = np.array([0, 125 , 150], np.uint8)
    red_upper = np.array([255, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Morphological Transform, Dilation for yellow color
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.dilate(red_mask, kernel)

    # Find contours
    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    X_distance = None
    # Draw circles around contours and calculate distance
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 2000:
            # Calculate the radius of the enclosing circle
            (x2, y2), radius = cv2.minEnclosingCircle(contour)
            center = (int(x2), int(y2))
            radius = int(radius)

            cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)
            cv2.putText(imageFrame, "Red Ball", center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255))

            # Calculate distance using the simplified formula
            apparent_diameter = radius * 2  # Apparent diameter of the ball in pixels
            z_distance = (real_ball_diameter * focal_length) / apparent_diameter
            cv2.putText(imageFrame, f"Z_Distance: {z_distance:.2f} Millimeters", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            X_distance = x2-x1
            print(X_distance)
            Y_distance = y2-y1
            cv2.putText(imageFrame, f"X_Distance: {X_distance:.2f} Millimeters", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 79, 255), 2)
            cv2.putText(imageFrame, f"Y_Distance: {Y_distance:.2f} Millimeters", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 100, 255), 2)
            if X_distance != 0:
                if X_distance > 0 :
                    moveleftdir()
                elif X_distance < 0:
                   moverightdir()
            elif X_distance < 40 and X_distance > -80 and z_distance != 0:
                if z_distance > 0:
                    movebackdir()
                if z_distance <= 0:
                    movefrontdir()
            elif z_distance == 0 and X_distance == 0:
                stop()

    if X_distance == None:
        print('No x2')
        stop()

       # switch on the gripper motor
            # Displaying the output
    cv2.imshow("Red Ball Detection in Real-Time", imageFrame)
    cv2.imshow("Red Ball Detection in Real-Time_hsv", hsvFrame)

    # Exiting the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
webcam.release()
cv2.destroyAllWindows()nce = (real_ball_diameter * focal_length) / apparent_diameter
            cv2.putText(imageFrame, f"Z_Distance: {z_distance:.2f} Millimeters", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            X_distance = x2-x1
            print(X_distance)
            Y_distance = y2-y1
            cv2.putText(imageFrame, f"X_Distance: {X_distance:.2f} Millimeters", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 79, 255), 2)
            cv2.putText(imageFrame, f"Y_Distance: {Y_distance:.2f} Millimeters", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 100, 255), 2)
            if X_distance != 0:
                if X_distance > 0 :
                    moveleftdir()
                elif X_distance < 0:
                   moverightdir()
            elif X_distance < 40 and X_distance > -80 and z_distance != 0:
                if z_distance > 0:
                    movebackdir()
                if z_distance <= 0:
                    movefrontdir()
            elif z_distance == 0 and X_distance == 0:
                stop()

    if X_distance == None:
        print('No x2')
        stop()

       # switch on the gripper motor
            # Displaying the output
    cv2.imshow("Red Ball Detection in Real-Time", imageFrame)
    cv2.imshow("Red Ball Detection in Real-Time_hsv", hsvFrame)

    # Exiting the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
webcam.release()
cv2.destroyAllWindows() Transform, Dilation for yellow color
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.dilate(red_mask, kernel)

    # Find contours
    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    X_distance = None
    # Draw circles around contours and calculate distance
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 2000:
            # Calculate the radius of the enclosing circle
            (x2, y2), radius = cv2.minEnclosingCircle(contour)
            center = (int(x2), int(y2))
            radius = int(radius)

            cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)
            cv2.putText(imageFrame, "Red Ball", center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255))

            # Calculate distance using the simplified formula
            apparent_diameter = radius * 2  # Apparent diameter of the ball in pixels
            z_distance = (real_ball_diameter * focal_length) / apparent_diameter
            cv2.putText(imageFrame, f"Z_Distance: {z_distance:.2f} Millimeters", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            X_distance = x2-x1
            print(X_distance)
            Y_distance = y2-y1
            cv2.putText(imageFrame, f"X_Distance: {X_distance:.2f} Millimeters", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 79, 255), 2)
            cv2.putText(imageFrame, f"Y_Distance: {Y_distance:.2f} Millimeters", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 100, 255), 2)
            if X_distance != 0:
                if X_distance > 0 :
                    moveleftdir()
                elif X_distance < 0:
                   moverightdir()
            elif X_distance < 40 and X_distance > -80 and z_distance != 0:
                if z_distance > 0:
                    movebackdir()
                if z_distance <= 0:
                    movefrontdir()
            elif z_distance == 0 and X_distance == 0:
                stop()

    if X_distance == None:
        print('No x2')
        stop()

       # switch on the gripper motor
            # Displaying the output
    cv2.imshow("Red Ball Detection in Real-Time", imageFrame)
    cv2.imshow("Red Ball Detection in Real-Time_hsv", hsvFrame)

    # Exiting the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
webcam.release()
cv2.destroyAllWindows()contourArea(contour)
        if area > 2000:
            # Calculate the radius of the enclosing circle
            (x2, y2), radius = cv2.minEnclosingCircle(contour)
            center = (int(x2), int(y2))
            radius = int(radius)

            cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)
            cv2.putText(imageFrame, "Red Ball", center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255))

            # Calculate distance using the simplified formula
            apparent_diameter = radius * 2  # Apparent diameter of the ball in pixels
            z_distance = (real_ball_diameter * focal_length) / apparent_diameter
            cv2.putText(imageFrame, f"Z_Distance: {z_distance:.2f} Millimeters", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            X_distance = x2-x1
            print(X_distance)
            Y_distance = y2-y1
            cv2.putText(imageFrame, f"X_Distance: {X_distance:.2f} Millimeters", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 79, 255), 2)
            cv2.putText(imageFrame, f"Y_Distance: {Y_distance:.2f} Millimeters", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 100, 255), 2)
            if X_distance != 0:
                if X_distance > 0 :
                    moveleftdir()
                elif X_distance < 0:
                   moverightdir()
            elif X_distance < 40 and X_distance > -80 and z_distance != 0:
                if z_distance > 0:
                    movebackdir()
                if z_distance <= 0:
                    movefrontdir()
            elif z_distance == 0 and X_distance == 0:
                stop()

    if X_distance == None:
        print('No x2')
        stop()

       # switch on the gripper motor
            # Displaying the output
    cv2.imshow("Red Ball Detection in Real-Time", imageFrame)
    cv2.imshow("Red Ball Detection in Real-Time_hsv", hsvFrame)

    # Exiting the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
webcam.release()
cv2.destroyAllWindows()
            # Calculate distance using the simplified formula
            apparent_diameter = radius * 2  # Apparent diameter of the ball in pixels
            z_distance = (real_ball_diameter * focal_length) / apparent_diameter
            cv2.putText(imageFrame, f"Z_Distance: {z_distance:.2f} Millimeters", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            X_distance = x2-x1
            print(X_distance)
            Y_distance = y2-y1
            cv2.putText(imageFrame, f"X_Distance: {X_distance:.2f} Millimeters", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 79, 255), 2)
            cv2.putText(imageFrame, f"Y_Distance: {Y_distance:.2f} Millimeters", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 100, 255), 2)
            if X_distance != 0:
                if X_distance > 0 :
                    moveleftdir()
                elif X_distance < 0:
                   moverightdir()
            elif X_distance < 40 and X_distance > -80 and z_distance != 0:
                if z_distance > 0:
                    movebackdir()
                if z_distance <= 0:
                    movefrontdir()
            elif z_distance == 0 and X_distance == 0:
                stop()

    if X_distance == None:
        print('No x2')
        stop()

       # switch on the gripper motor
            # Displaying the output
    cv2.imshow("Red Ball Detection in Real-Time", imageFrame)
    cv2.imshow("Red Ball Detection in Real-Time_hsv", hsvFrame)

    # Exiting the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
webcam.release()
cv2.destroyAllWindows()contourArea(contour)
        if area > 2000:
            # Calculate the radius of the enclosing circle
            (x2, y2), radius = cv2.minEnclosingCircle(contour)
            center = (int(x2), int(y2))
            radius = int(radius)

            cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)
            cv2.putText(imageFrame, "Red Ball", center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255))

            # Calculate distance using the simplified formula
            apparent_diameter = radius * 2  # Apparent diameter of the ball in pixels
            z_distance = (real_ball_diameter * focal_length) / apparent_diameter
            cv2.putText(imageFrame, f"Z_Distance: {z_distance:.2f} Millimeters", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            X_distance = x2-x1
            print(X_distance)
            Y_distance = y2-y1
            cv2.putText(imageFrame, f"X_Distance: {X_distance:.2f} Millimeters", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 79, 255), 2)
            cv2.putText(imageFrame, f"Y_Distance: {Y_distance:.2f} Millimeters", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 100, 255), 2)
            if X_distance != 0:
                if X_distance > 0 :
                    moveleftdir()
                elif X_distance < 0:
                   moverightdir()
            elif X_distance < 40 and X_distance > -80 and z_distance != 0:
                if z_distance > 0:
                    movebackdir()
                if z_distance <= 0:
                    movefrontdir()
            elif z_distance == 0 and X_distance == 0:
                stop()

    if X_distance == None:
        print('No x2')
        stop()

       # switch on the gripper motor
            # Displaying the output
    cv2.imshow("Red Ball Detection in Real-Time", imageFrame)
    cv2.imshow("Red Ball Detection in Real-Time_hsv", hsvFrame)

    # Exiting the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
webcam.release()
cv2.destroyAllWindows()array([255, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Morphological Transform, Dilation for yellow color
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.dilate(red_mask, kernel)

    # Find contours
    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    X_distance = None
    # Draw circles around contours and calculate distance
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 2000:
            # Calculate the radius of the enclosing circle
            (x2, y2), radius = cv2.minEnclosingCircle(contour)
            center = (int(x2), int(y2))
            radius = int(radius)

            cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)
            cv2.putText(imageFrame, "Red Ball", center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255))

            # Calculate distance using the simplified formula
            apparent_diameter = radius * 2  # Apparent diameter of the ball in pixels
            z_distance = (real_ball_diameter * focal_length) / apparent_diameter
            cv2.putText(imageFrame, f"Z_Distance: {z_distance:.2f} Millimeters", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            X_distance = x2-x1
            print(X_distance)
            Y_distance = y2-y1
            cv2.putText(imageFrame, f"X_Distance: {X_distance:.2f} Millimeters", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 79, 255), 2)
            cv2.putText(imageFrame, f"Y_Distance: {Y_distance:.2f} Millimeters", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 100, 255), 2)
            if X_distance != 0:
                if X_distance > 0 :
                    moveleftdir()
                elif X_distance < 0:
                   moverightdir()
            elif X_distance < 40 and X_distance > -80 and z_distance != 0:
                if z_distance > 0:
                    movebackdir()
                if z_distance <= 0:
                    movefrontdir()
            elif z_distance == 0 and X_distance == 0:
                stop()

    if X_distance == None:
        print('No x2')
        stop()

       # switch on the gripper motor
            # Displaying the output
    cv2.imshow("Red Ball Detection in Real-Time", imageFrame)
    cv2.imshow("Red Ball Detection in Real-Time_hsv", hsvFrame)

    # Exiting the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
webcam.release()
cv2.destroyAllWindows()contourArea(contour)
        if area > 2000:
            # Calculate the radius of the enclosing circle
            (x2, y2), radius = cv2.minEnclosingCircle(contour)
            center = (int(x2), int(y2))
            radius = int(radius)

            cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)
            cv2.putText(imageFrame, "Red Ball", center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255))

            # Calculate distance using the simplified formula
            apparent_diameter = radius * 2  # Apparent diameter of the ball in pixels
            z_distance = (real_ball_diameter * focal_length) / apparent_diameter
            cv2.putText(imageFrame, f"Z_Distance: {z_distance:.2f} Millimeters", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            X_distance = x2-x1
            print(X_distance)
            Y_distance = y2-y1
            cv2.putText(imageFrame, f"X_Distance: {X_distance:.2f} Millimeters", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 79, 255), 2)
            cv2.putText(imageFrame, f"Y_Distance: {Y_distance:.2f} Millimeters", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 100, 255), 2)
            if X_distance != 0:
                if X_distance > 0 :
                    moveleftdir()
                elif X_distance < 0:
                   moverightdir()
            elif X_distance < 40 and X_distance > -80 and z_distance != 0:
                if z_distance > 0:
                    movebackdir()
                if z_distance <= 0:
                    movefrontdir()
            elif z_distance == 0 and X_distance == 0:
                stop()

    if X_distance == None:
        print('No x2')
        stop()

       # switch on the gripper motor
            # Displaying the output
    cv2.imshow("Red Ball Detection in Real-Time", imageFrame)
    cv2.imshow("Red Ball Detection in Real-Time_hsv", hsvFrame)

    # Exiting the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
webcam.release()
cv2.destroyAllWindows()), radius = cv2.minEnclosingCircle(contour)
            center = (int(x2), int(y2))
            radius = int(radius)
            cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)
            cv2.putText(imageFrame, "Red Ball", center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255))

            # Calculate distance using the simplified formula
            apparent_diameter = radius * 2  # Apparent diameter of the ball in pixels
            z_distance = (real_ball_diameter * focal_length) / apparent_diameter
            cv2.putText(imageFrame, f"Distance: {z_distance:.2f} Millimeters", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            y_distance = x2-x1
            x_distance = y2-y1
            cv2.putText(imageFrame, f"Distance: {y_distance:.2f} Millimeters", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 79, 255), 2)
            cv2.putText(imageFrame, f"Distance: {x_distance:.2f} Millimeters", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 100, 255), 2)
            # Displaying the output
    cv2.imshow("Red Ball Detection in Real-Time", imageFrame)

    # Exiting the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
webcam.release()
cv2.destroyAllWindows()