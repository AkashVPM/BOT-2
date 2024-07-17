import RPi.GPIO as GPIO
import time

# Define GPIO pins for stepper motor connections
pin1 = 31 # GPIO17 (pin 6)
pin2 = 33 # GPIO18 (pin 13)
pin3 = 35 # GPIO27 (pin 19)
pin4 = 37 # GPIO22 (pin 26)

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin1, GPIO.OUT)
GPIO.setup(pin2, GPIO.OUT)
GPIO.setup(pin3, GPIO.OUT)
GPIO.setup(pin4, GPIO.OUT)

# Define the number of steps per revolution and speed
steps_per_revolution = 100
delay = 0.001  # Adjust this value to control the speed

# Function to rotate stepper motor
def rotate_stepper(direction, steps):
    # Define the step sequence for half-stepping
    half_step_sequence = [
        [1, 0, 0, 0],
        [1, 1, 0, 0],
        [0, 1, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 1, 0],
        [0, 0, 1, 1],
        [0, 0, 0, 1],
        [1, 0, 0, 1]
    ]

    # Set direction
    if direction == "forward":
        sequence = half_step_sequence
    elif direction == "backward":
        sequence = list(reversed(half_step_sequence))

    # Step the motor
    for _ in range(steps):
        for halfstep in sequence:
            GPIO.output(pin1, halfstep[0])
            GPIO.output(pin2, halfstep[1])
            GPIO.output(pin3, halfstep[2])
            GPIO.output(pin4, halfstep[3])
            time.sleep(delay)

try:
    # Forward rotation for 5 revolutions
    rotate_stepper("forward", 15 * steps_per_revolution)

    # Backward rotation for 5 revolutions
    rotate_stepper("backward", 15 * steps_per_revolution)

except KeyboardInterrupt:
    pass

finally:
    # Clean up GPIO
    GPIO.cleanup()