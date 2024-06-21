# line follower 
# Color detection(Red, blue, yellow)


import cv2
import numpy as np
import RPi.GPIO as GPIO
from time import sleep
import threading
import smbus #comunicación I2C
import math
from gpiozero import Button
from signal import pause 
# import numpy as np

# locomotion comment

GPIO.setmode (GPIO.BCM)
GPIO.setwarnings (False)

AN1 = 24   #locomotion 
AN2 = 25
AN3 = 17
AN4 = 27
AN5 = 11   # for conveyor motor 
AN6 = 8    # for gripper /motor

DIG1 = 18  #locomotion
DIG2 = 23
DIG3 = 22
DIG4 = 5
DIG5 = 15   # conveyor motor
DIG6 = 7    # for gripper motor 

limitswitch_1 = 29   # for limitswitch 1 (conveyor at top ) 
limitswitch_2 = 13   # for limitswitch 2 (conveyor at bottom)  
limitswitch_3 = 16   # for gripper limit switch 
limitswitch_4 = 12   # for grippefr limit switch

GPIO.setup(AN1, GPIO.OUT)
GPIO.setup(AN2, GPIO.OUT)
GPIO.setup(AN3, GPIO.OUT)
GPIO.setup(AN4, GPIO.OUT)
GPIO.setup(AN5, GPIO.OUT)
GPIO.setup(AN6, GPIO.OUT)

GPIO.setup(DIG1, GPIO.OUT)
GPIO.setup(DIG2, GPIO.OUT)
GPIO.setup(DIG3, GPIO.OUT)
GPIO.setup(DIG4, GPIO.OUT)
GPIO.setup(DIG5, GPIO.OUT)
GPIO.setup(DIG6, GPIO.OUT)

GPIO.setup(limitswitch_1, GPIO.IN,pull_up_down = GPIO.PUD_up)
GPIO.setup(limitswitch_2, GPIO.IN,pull_up_down = GPIO.PUD_up)
GPIO.setup(limitswitch_3, GPIO.IN,pull_up_down = GPIO.PUD_up)
GPIO.setup(limitswitch_4, GPIO.IN,pull_up_down = GPIO.PUD_up)

sleep (1)

p1 = GPIO.PWM(AN1, 255)
p2 = GPIO.PWM(AN2, 255)
p3 = GPIO.PWM(AN3, 255)
p4 = GPIO.PWM(AN4, 255)
p5 = GPIO.PWM(AN5, 255)
p6 = GPIO.PWM(AN6, 255)

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
    GPIO.output(DIG2, GPIO.LOW)
    GPIO.output(DIG3, GPIO.LOW)
    GPIO.output(DIG4, GPIO.HIGH)
    p1.start(0)
    p2.start(0)
    p3.start(0)
    p4.start(0)

real_ball_diameter = 195  # Diameter of the ball in meters (10 cm)
focal_length = 1000       # Focal length of the camera in pixels (example value)


# initial color detection, decide our team color

Team_Red = 0
Team_Blue = 0
yellow_zone = 0

def find_dominant_color(frame):
    # Separating the frame into color components (blue, green, red)
    b, g, r = cv2.split(frame)
    # Calculating the mean value for each color component
    b_mean = np.mean(b)
    g_mean = np.mean(g)
    r_mean = np.mean(r)
    if max(b_mean, g_mean, r_mean) == r_mean:
        Team_Red = 1 
        Team_Blue = 0
    elif max(b_mean, g_mean, r_mean) == g_mean:
        Team_Blue = 0
        Team_Red = 0
    elif (g_mean >= 150 and r_mean >= 150) or (r_mean >= 200  and g_mean >= 100):
        yellow_zone = 1
    else:
        Team_Blue = 1
        Team_Red = 0

webcam = cv2.VideoCapture(0)
while True:
        # Reading the current frame from the webcam
        _, imageFrame = webcam.read()
        # Finding the dominant color in the current frame
        dominant_color = find_dominant_color(imageFrame)  # dominant_Color is just used for call find_dominanit_color function

        if (Team_Red == 1 or Team_Blue == 1):
            break

sleep(2)

# line follower follows white line 

def zeroloop():
    video_capture = cv2.VideoCapture(0)
    video_capture.set(3, 50)
    video_capture.set(4,60)

    while(True):
       ret, frame = video_capture.read()
       crop_img = frame[10:300, 10:300]

    #   gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    #   blur = cv2.GaussianBlur(gray, (5, 5), 0)
    #   ret, thresh = cv2.threshold(blur,200, 255,cv2.THRESH_BINARY)
    #   Convert frame to HSV color space

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
# exit after 5 sec of yellow detection
#    if cv2.waitKey(1) & 0xFF == ord('q'):
       if ( yellow_zone == 1):
            start_time = threading.Event          
            start_time.wait(7)
            break # need to change the condition 

zeroloop()


# Capturing video through webcam
# If we are team Red this code will activate

if Team_Red == 1:
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
                    else:
                        moverightdir()
                elif X_distance < 40 and X_distance > -80 and z_distance != 0:
                    if z_distance > 0:
                        movebackdir()
                    else:
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
        if ( yellow_zone == 1):
            start_time = threading.Event          
            start_time.wait(7)
            break

# if we are team Blue this code will activate

if Team_Blue == 1:
    while True:
    # Reading the video from the webcam in image frames
        _, imageFrame = webcam.read()

        # Convert the imageFrame from BGR (RGB color space) to HSV (hue-saturation-value) color space
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
        # Set range for yellow color and define mask
        rows, cols = imageFrame.shape[:2]
        x1 = rows/2
        y1 = cols/2

        blue_lower = np.array([45.86, 183.09, 182.58], np.uint8)
        blue_upper = np.array([255, 255, 255], np.uint8)
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

        # Morphological Transform, Dilation for yellow color

        # Find contours
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Morphological Transform, Dilation for yellow color
        kernel = np.ones((5, 5), np.uint8)
        blue_mask = cv2.dilate(blue_mask, kernel)

        # Find contours
        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw circles around contours and calculate distance
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 2000:
                # Calculate the radius of the enclosing circle
                (x2, y2), radius = cv2.minEnclosingCircle(contour)
                center = (int(x2), int(y2))
                radius = int(radius)
                cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)
                cv2.putText(imageFrame, "Blue Ball", center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255))

                # Calculate distance using the simplified formula
                apparent_diameter = radius * 2  # Apparent diameter of the ball in pixels
                z_distance = (real_ball_diameter * focal_length) / apparent_diameter
                cv2.putText(imageFrame, f"Z_Distance: {z_distance:.2f} Millimeters", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                X_distance = x2-x1
                Y_distance = y2-y1
                cv2.putText(imageFrame, f"X_Distance: {X_distance:.2f} Millimeters", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 79, 255), 2)
                cv2.putText(imageFrame, f"Y_Distance: {Y_distance:.2f} Millimeters", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 100, 255), 2)

            # switch on the gripper motor
            # Displaying the output

        cv2.imshow("Blue Ball Detection in Real-Time", imageFrame)
        cv2.imshow("Blue Ball Detection in Real-Time_hsv", hsvFrame)

        # Exiting the loop if yellow detected
        if ( yellow_zone == 1):
            start_time = threading.Event          
            start_time.wait(7)
            break


# area 3 
if (ball == 0 and yellow_zone == 1):  # turn left slowly 
    GPIO.output(DIG1, GPIO.LOW)
    GPIO.output(DIG2, GPIO.LOW)
    GPIO.output(DIG3, GPIO.HIGH)
    GPIO.output(DIG4, GPIO.HIGH)
    p1.start(35)
    p2.start(35)
    p3.start(35)
    p4.start(35)

if ( ball == 1 and yellow_zone == 1):
        stop()

if (ball == 1 and X_distance <= 200):
    movefrontdir()

if (ball == 1 and X_distance < 200):
    stop()

if ((GPIO.input(limitswitch_1)== 1)):         # conveyor upper limit switch 
    if ((GPIO.input(limitswitch_3)== 0 and GPIO.input(limitswitch_4)== 0)): # gripper limit switch off 
        sleep(5)
        GPIO.output(DIG5, GPIO.LOW)
        p5.start(75)
    if ((GPIO.input(limitswitch_3)== 1 and GPIO.input(limitswitch_4)== 1)): # gripper limit switch on 
        GPIO.output(DIG5, GPIO.LOW)
        p5.start(0)

if ((GPIO.input(limitswitch2)== 1)):          # conveyor lower limit switch
    if ((GPIO.input(limitswitch_3)== 1 and GPIO.input(limitswitch_4)== 1)): # gripper limit switch off 
        GPIO.output(DIG5, GPIO.HGH)
        p5.start(75)
    if ((GPIO.input(limitswitch_3)== 0 and GPIO.input(limitswitch_4)== 0)): # gripper limit switch on 
        GPIO.output(DIG5, GPIO.LOW)
        p5.start(0)

if (X_distance < 200 and (GPIO.input(limitswitch_3)== 0) and (GPIO.input(limitswitch_4)== 0)): # close gripper 
    GPIO.output(DIG6, GPIO.LOW)
    p6.start(75)

if ((GPIO.input(limitswitch_3)== 1) and (GPIO.input(limitswitch_4)== 1)): # stop gripper 
    GPIO.output(DIG6, GPIO.LOW)
    p6.start(0)




# compass sensor 

#direcciones requeridas
#---------------config
Registro_A = 0x0B
Registro_B = 0x09
RegStatus = 0x06
RegCtrl = 0x09
#---------------direcciones de la conexión
bus=smbus.SMBus(1)
deviceAdress = 0x0d
#---------------datos
eje_X_Mag = 0x00
eje_Y_Mag = 0x02
eje_Z_Mag = 0x04
declination = -0.00669
pi=3.14159265359


def MagnetometerInit():
#configurar registro A
    bus.write_byte_data(deviceAdress, Registro_A, 0x01)
#configurar registro B
    bus.write_byte_data(deviceAdress, Registro_B, 0x1D)
#configurar registro para seleccionar el modo
#bus.write_byte_data(deviceAdress, ModoRegistro, 0)

def read_raw_data(addr):
#leer doble byte (16 bits)
    low = bus.read_byte_data(deviceAdress,addr)
    high = bus.read_byte_data(deviceAdress,addr+1)
#concatenar los bytes
    valor = ((high << 8) | low)
#obtener el signo
    if(valor > 32768):
        valor = valor - 65536
    return valor
#------------------------------------------main
MagnetometerInit()
print('leyendo magnetometro...')
while True:
    bandera = bus.read_byte_data(deviceAdress,RegStatus)
    a="{0:b}".format(bandera)
    if a[len(a)-1] == 0:
        bandera = bus.read_byte_data(deviceAdress,RegStatus)
    x=read_raw_data(eje_X_Mag)
    y=read_raw_data(eje_Y_Mag)
    z=read_raw_data(eje_Z_Mag)
    heading = math.atan2(y,x)+ declination
#compensar superiores a 360
    if(heading > 2*pi):
        heading = heading -2*pi
#revisar el signo
    if(heading < 0):
        heading=heading+2*pi
#convertir a grados
    heading_angle = int(heading * (180/pi))
    print("angulo = %d°" %heading_angle)
    sleep(0.5)

# turn the bot left
# if (heading_angle >= 80 and heading_angle <= 95):
    



# Release the webcam, close the window and turn off the bot 

webcam.release()
cv2.destroyAllWindows()
GPIO.output(DIG2, GPIO.HIGH)
GPIO.output(DIG3, GPIO.LOW)
GPIO.output(DIG4, GPIO.LOW)
p1.start(0)
p2.start(0)
p3.start(0)
p4.start(0)