# Run sudo pigpiod in terminal before running code

from gpiozero import Button, DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from time import time, sleep
import pigpio
import numpy as np
import cv2
from picamera2 import Picamera2

button = Button(13, bounce_time=0.2, pull_up=True)

def wait_for_start():
    print("Waiting...")
    button.wait_for_press()
    print("Robot Started!")

# --- Wait for start ---
wait_for_start()

# --- Setup ---
pi = pigpio.pi()
turn_amt = -200
base_speed = 100
offset = 0

# Ultrasonic sensors
factory = PiGPIOFactory()
frontultrasonic = DistanceSensor(echo=5, trigger=6, max_distance=4, pin_factory=factory)
leftultrasonic = DistanceSensor(echo=17, trigger=4, max_distance=4, pin_factory=factory)
rightultrasonic = DistanceSensor(echo=27, trigger=22, max_distance=4, pin_factory=factory)
backultrasonic = DistanceSensor(echo=19, trigger=26, max_distance=4, pin_factory=factory)

# Servo
servo_pin = 16
pwm = pigpio.pi()
pwm.set_mode(servo_pin, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo_pin, 50)
pwm.set_servo_pulsewidth(servo_pin, 1500 + offset)

# Camera
cam = Picamera2()
cam.configure(cam.create_preview_configuration({"size": (640, 360)}))
cam.start()

# Motors
M1A, M1B = 23, 24
pi.set_mode(M1A, pigpio.OUTPUT)
pi.set_mode(M1B, pigpio.OUTPUT)

# --- Motor control ---
def motorSpeed(speed):
    if speed < 0:
        pi.set_PWM_dutycycle(M1A, abs(speed))
        pi.set_PWM_dutycycle(M1B, 0)
    elif speed > 0:
        pi.set_PWM_dutycycle(M1A, 0)
        pi.set_PWM_dutycycle(M1B, abs(speed))
    else:
        pi.set_PWM_dutycycle(M1A, 0)
        pi.set_PWM_dutycycle(M1B, 0)

# --- Orange line detection ---
last_orange_seen = -1
def detect_orange(frame, on_orange_line, lap_count):
    global last_orange_seen
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    orange_lower = np.array([5, 150, 150], np.uint8)
    orange_upper = np.array([20, 255, 255], np.uint8)
    orange_mask = cv2.inRange(hsvFrame, orange_lower, orange_upper)

    height, width = orange_mask.shape
    roi = orange_mask[height-60:height, :]
    orange_pixels = cv2.countNonZero(roi)

    ctime = time()
    crossed = False
    if orange_pixels > 600:
        if not on_orange_line and (ctime - last_orange_seen) > 1.6:
            lap_count += 1
            crossed = True
            on_orange_line = True
            last_orange_seen = ctime
        elif (ctime - last_orange_seen) <= 1.6:
            print("Orange double count prevented")
    else:
        on_orange_line = False

    return lap_count, on_orange_line, crossed

# --- Blue line detection ---
last_blue_seen = -1
def detect_blue(frame, on_blue_line, lap_count):
    global last_blue_seen
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blue_lower = np.array([100, 150, 20], np.uint8)
    blue_upper = np.array([130, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    height, width = blue_mask.shape
    roi = blue_mask[height-60:height, :]
    blue_pixels = cv2.countNonZero(roi)

    ctime = time()

    crossed = False
    if blue_pixels > 600:
        if not on_blue_line and (ctime - last_blue_seen) > 1.6:
            lap_count += 1
            crossed = True
            on_blue_line = True
            last_blue_seen = ctime
        elif (ctime - last_blue_seen) <= 1.6:
            print("Blue double count prevented")
    else:
        on_blue_line = False

    return lap_count, on_blue_line, crossed


# --- Main Loop ---
lap_count = 0
on_orange_line = False
orange_sequence = None  
is_orange = 0

try:
    while True:
        frame = cv2.cvtColor(cam.capture_array()[::-1, :, :3], cv2.COLOR_RGB2BGR)
        distance_cm = frontultrasonic.distance * 100
        current_time = time()

        if lap_count < 12:
            crossed = False
            if is_orange == 0:
                lap_count, on_orange_line, crossed = detect_orange(frame, on_orange_line, lap_count)
                if lap_count == 1:
                    is_orange = 1
                    turn_amt = 200
                else:
                    lap_count, on_orange_line, crossed = detect_blue(frame, on_orange_line, lap_count)
                    if lap_count == 1:
                        is_orange = -1
                        turn_amt = -200
            elif is_orange == 1:
                lap_count, on_orange_line, crossed = detect_orange(frame, on_orange_line, lap_count)
            elif is_orange == -1:
                lap_count, on_orange_line, crossed = detect_blue(frame, on_orange_line, lap_count)

            motorSpeed(base_speed)

            if crossed and orange_sequence is None:
                orange_sequence = ([(1500 + offset, 0.0), ((1500 + offset + turn_amt), 1.60), (1500 + offset, 0)], 0, current_time)
                print(f"Lap {lap_count} completed")

            if orange_sequence is not None:
                steps, step_index, start_time = orange_sequence
                pos, duration = steps[step_index]
                pwm.set_servo_pulsewidth(servo_pin, pos)
                if current_time - start_time >= duration:
                    step_index += 1
                    if step_index >= len(steps):
                        orange_sequence = None
                    else:
                        orange_sequence = (steps, step_index, current_time)

            if orange_sequence is None:
                left_dist = leftultrasonic.distance * 100
                right_dist = rightultrasonic.distance * 100
                print("Servo distances: ", int(left_dist), int(right_dist))
                if left_dist < 32:
                    pwm.set_servo_pulsewidth(servo_pin, (1500 + offset + 300))
                    motorSpeed(100)
                elif right_dist < 32:
                    pwm.set_servo_pulsewidth(servo_pin, (1500 + offset - 300))
                    motorSpeed(100)
                else:
                    pwm.set_servo_pulsewidth(servo_pin, 1500 + offset)


        else:
            pwm.set_servo_pulsewidth(servo_pin, 1500 + offset + turn_amt)
            sleep(2.5)
            pwm.set_servo_pulsewidth(servo_pin, 1500 + offset)
            sleep(1.5)
            motorSpeed(0)
            break

        if button.is_pressed:
            motorSpeed(0)
            pwm.set_servo_pulsewidth(servo_pin, 1500)
            sleep(0.1)
            pwm.set_servo_pulsewidth(servo_pin, 0)
            break

        if cv2.waitKey(10) & 0xFF == ord('q'):
            motorSpeed(0)
            break

finally:
    motorSpeed(0)
    pwm.set_servo_pulsewidth(servo_pin, 0)
    cv2.destroyAllWindows()
