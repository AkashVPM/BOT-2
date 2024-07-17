import cv2
import numpy as np
import RPi.GPIO as GPIO
from time import sleep
import threading

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
    print("Moving front")
    GPIO.output(DIG1, GPIO.HIGH)
    GPIO.output(DIG2, GPIO.LOW)
    GPIO.output(DIG3, GPIO.LOW)
    GPIO.output(DIG4, GPIO.HIGH)
    for pwm in [p1, p2, p3, p4]:
        pwm.start(100)

def movebackdir():
    print("Moving back")
    GPIO.output(DIG1, GPIO.LOW)
    GPIO.output(DIG2, GPIO.HIGH)
    GPIO.output(DIG3, GPIO.HIGH)
    GPIO.output(DIG4, GPIO.LOW)
    for pwm in [p1, p2, p3, p4]:
        pwm.start(100)

def moveleftdir():
    print("Moving left")
    GPIO.output(DIG1, GPIO.LOW)
    GPIO.output(DIG2, GPIO.LOW)
    GPIO.output(DIG3, GPIO.HIGH)
    GPIO.output(DIG4, GPIO.HIGH)
    for pwm in [p1, p2, p3, p4]:
        pwm.start(100)

def moverightdir():
    print("Moving right")
    GPIO.output(DIG1, GPIO.HIGH)
    GPIO.output(DIG2, GPIO.HIGH)
    GPIO.output(DIG3, GPIO.LOW)
    GPIO.output(DIG4, GPIO.LOW)
    for pwm in [p1, p2, p3, p4]:
        pwm.start(100)

def stop():
    print("Stopping")
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
            print("Team color set to Red")
        elif max(b_mean, g_mean, r_mean) == g_mean:
            Team_Blue, Team_Red = 1, 0
            blue_set = True
            print("Team color set to Blue")
        elif (g_mean >= 150 and r_mean >= 150) or (r_mean >= 200 and g_mean >= 100):
            yellow_zone = 1
            yellow_set = True
            print("Yellow zone detected")
        else:
            Team_Blue, Team_Red = 1, 0
    
    # Yellow zone update
    if not yellow_set:
        if (g_mean >= 150 and r_mean >= 150) or (r_mean >= 200 and g_mean >= 100):
            yellow_zone = 1
            yellow_set = True
            print("Yellow zone detected and updated")
        else:
            yellow_zone = 0
            print("Yellow zone not detected")

# Example usage:
# frame = cv2.imread('path_to_your_image.jpg')
# find_dominant_color(frame)

# Initialize webcam
webcam = cv2.VideoCapture(0)
while True:
    _, imageFrame = webcam.read()
    find_dominant_color(imageFrame)
    if Team_Red == 1 or Team_Blue == 1:
        print("Team color identified, breaking loop")
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
                print(f"Center X: {cx}")
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
                print("Zero moment detected, restarting zeroloop")
                zeroloop()
        else:
            print("I don't see the line")
            stop()

        cv2.imshow('frame', crop_img)
        cv2.imshow('frame1', hsv_frame)
        # Exit after 7 sec of yellow detection
        if yellow_zone == 1:
            print("Yellow zone detected, waiting 7 seconds before breaking")
            start_time = threading.Event()
            start_time.wait(7)
            break  # Need to change the condition

# Initialize and start the line following loop
print("Starting zeroloop")
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
                if radius > 10:
                    cv2.circle(imageFrame, (int(x2), int(y2)), int(radius), (0, 255, 255), 2)
                    center = (int(x2), int(y2))
                    cv2.putText(imageFrame, f"{center}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    X_distance = abs(x1 - x2)
                    cv2.line(imageFrame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

                    if X_distance <= focal_length:
                        print("Ball within reach, stopping movement")
                        stop()
                        sleep(2)
                        print("Picking up the ball")
                        # Add mechanism to pick up the ball here
                        sleep(2)
                        print("Delivering the ball")
                        # Add mechanism to deliver the ball here
                        sleep(2)
                        break
                    else:
                        print("Moving towards the ball")
                        # Adjust movement towards the ball here

        cv2.imshow("Ball Detection", imageFrame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Start ball detection based on team color
if Team_Red == 1:
    print("Starting ball detection for Red team")
    ball_detection("red")
elif Team_Blue == 1:
    print("Starting ball detection for Blue team")
    ball_detection("blue")

# Clean up GPIO and camera resources
GPIO.cleanup()
cv2.destroyAllWindows()
