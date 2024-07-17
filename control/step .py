import RPi.GPIO as GPIO
import time

# Define GPIO pins for L298N driver
IN1 = 6
IN2 = 13
IN3 = 19
IN4 = 26

# Set GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Constants for stepper motor control
DEG_PER_STEP = 1.8
STEPS_PER_REVOLUTION = int(360 / DEG_PER_STEP)

# Function to move the stepper motor one step forward
def step_forward(delay, steps):
    for _ in range(steps):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        time.sleep(delay)

        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        time.sleep(delay)

# Function to move the stepper motor one step backward
def step_backward(delay, steps):
    for _ in range(steps):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.HIGH)
        time.sleep(delay)

        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        time.sleep(delay)

try:
    # Set the delay between steps
    delay = 0.001

    # Perform forward rotation for one revolution (360 degrees)
    step_forward(delay, STEPS_PER_REVOLUTION)

    # Pause for 5 seconds
    time.sleep(5)

    # Perform backward rotation for one revolution (360 degrees)
    step_backward(delay, STEPS_PER_REVOLUTION)

except KeyboardInterrupt:
    print("\nExiting the script.")

finally:
    # Clean up GPIO settings
    GPIO.cleanup()