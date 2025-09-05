# Run sudo pigpiod in terminal before running code

from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from time import time
import pigpio
import numpy as np
import cv2
from picamera2 import Picamera2

pi = pigpio.pi()

# Ultrasonic sensors
factory = PiGPIOFactory()
frontultrasonic = DistanceSensor(echo=27, trigger=22, max_distance=4, pin_factory=factory) #White (22) and Brown (27)
leftultrasonic = DistanceSensor(echo=17, trigger=4, max_distance=4, pin_factory=factory) #Orange (17) and Blue (4)
rightultrasonic = DistanceSensor(echo=5, trigger=6, max_distance=4, pin_factory=factory) #Green (6) and Blue (5)
backultrasonic = DistanceSensor(echo=19, trigger=26, max_distance=4, pin_factory=factory) #white (19) and purple (26)q

# Servo
servo_pin = 16
pwm = pigpio.pi()
pwm.set_mode(servo_pin, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo_pin, 50)
pwm.set_servo_pulsewidth(servo_pin, 1500)  # neutral start

# Camera
cam = Picamera2()
cam.configure(cam.create_preview_configuration({"size": (640, 360)}))
cam.start()

# Motors
M1A = 23
M1B = 24
pi.set_mode(M1A, pigpio.OUTPUT)
pi.set_mode(M1B, pigpio.OUTPUT)

# Orange line detection
def detect_orange(frame, on_orange_line, lap_count):
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    orange_lower = np.array([5, 150, 150], np.uint8)
    orange_upper = np.array([20, 255, 255], np.uint8)
    orange_mask = cv2.inRange(hsvFrame, orange_lower, orange_upper)

    height, width = orange_mask.shape
    roi = orange_mask[height-60:height, :]
    orange_pixels = cv2.countNonZero(roi)

    crossed = False
    if orange_pixels > 1000:
        if not on_orange_line:
            lap_count += 1
            crossed = True
            on_orange_line = True
    else:
        on_orange_line = False

    contours, _ = cv2.findContours(orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 300:
            x, y, w, h = cv2.boundingRect(largest)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 165, 255), 2)

    return lap_count, on_orange_line, crossed

# Motor speed
def motorSpeed(speed):
    if speed > 0:
        pi.set_PWM_dutycycle(M1A, speed)
        pi.set_PWM_dutycycle(M1B, 0)
    elif speed < 0:
        pi.set_PWM_dutycycle(M1A, 0)
        pi.set_PWM_dutycycle(M1B, abs(speed))
    else:
        pi.set_PWM_dutycycle(M1A, 0)
        pi.set_PWM_dutycycle(M1B, 0)

# Main loop variables
lap_count = 0
on_orange_line = False
orange_sequence = None  # (steps, current_step, start_time)
servo_sequence = None  
stable_counter = 0  

try:
    while True:
        frame = cv2.cvtColor(cam.capture_array()[::-1, :, :3], cv2.COLOR_RGB2BGR)
        distance_cm = frontultrasonic.distance * 100
        current_time = time()


        if lap_count < 13:
            lap_count, on_orange_line, crossed = detect_orange(frame, on_orange_line, lap_count)

            if crossed and orange_sequence is None:
                orange_sequence = ([(1500,3), (1000,3), (1500,0)], 0, current_time)  # Move forward 2s, turn 2s, straighten
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

        if cv2.waitKey(10) & 0xFF == ord('q'):
            motorSpeed(0)
            break

finally:
    motorSpeed(0)
    pwm.set_servo_pulsewidth(servo_pin, 0)
    cv2.destroyAllWindows()