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
frontultrasonic = DistanceSensor(echo=27, trigger=22, max_distance=4, pin_factory=factory)
leftultrasonic = DistanceSensor(echo=17, trigger=4, max_distance=4, pin_factory=factory)
rightultrasonic = DistanceSensor(echo=5, trigger=6, max_distance=4, pin_factory=factory)
backultrasonic = DistanceSensor(echo=19, trigger=26, max_distance=4, pin_factory=factory)

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

# Red and green detection
def traffic_lights(frame):
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red
    red_lower1 = np.array([0, 150, 50], np.uint8)
    red_upper1 = np.array([5, 255, 255], np.uint8)
    red_lower2 = np.array([172, 150, 100], np.uint8)
    red_upper2 = np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower1, red_upper1) | cv2.inRange(hsvFrame, red_lower2, red_upper2)

    # Green
    green_lower = np.array([35, 50, 50], np.uint8)
    green_upper = np.array([85, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    kernel = np.ones((5, 5), "uint8")
    red_mask = cv2.dilate(red_mask, kernel)
    green_mask = cv2.dilate(green_mask, kernel)

    def detect_and_label(mask, color_bgr, label):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0, None
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if 30000 < area < 60000:  # stable area range
            x, y, w, h = cv2.boundingRect(largest)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 2)
            return area, label
        return 0, None

    red_area, red_label = detect_and_label(red_mask, (0, 0, 255), "Red")
    green_area, green_label = detect_and_label(green_mask, (0, 255, 0), "Green")

    return red_area, green_area, red_label, green_label

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

# --- MAIN LOOP ---
lap_count = 0
on_orange_line = False
orange_sequence = None  
servo_sequence = None  
stable_counter = 0  

try:
    while True:
        frame = cv2.cvtColor(cam.capture_array()[::-1, :, :3], cv2.COLOR_RGB2BGR)
        distance_cm = frontultrasonic.distance * 100
        current_time = time()

        if lap_count < 12: 
            lap_count, on_orange_line, crossed = detect_orange(frame, on_orange_line, lap_count)
            motorSpeed(100)

            if crossed and orange_sequence is None:
                orange_sequence = ([(1500, 1.5), (2000, 1), (1500,0)], 0, current_time)
                print(f"Lap {lap_count} completed")

            # --- handle orange sequence ---
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

            # ----- TRAFFIC LIGHT DETECTION -----
            if orange_sequence is None:
                red_area, green_area, red_label, green_label = traffic_lights(frame)

                if red_area > green_area or green_area > red_area:
                    stable_counter += 1
                else:
                    stable_counter = 0

                if servo_sequence is None and stable_counter >= 3:  # wait 3 frames
                    if red_area > green_area:
                        servo_sequence = ([(1000,1), (1500,1), (2000,1), (1500,0)], 0, current_time + 0.1)
                        color_detected = red_label
                    elif green_area > red_area:
                        servo_sequence = ([(2000,1), (1500,1), (1000,1), (1500,0)], 0, current_time + 0.1)
                        color_detected = green_label
                    else:
                        pwm.set_servo_pulsewidth(servo_pin, 1500)
                        color_detected = "None"
                elif servo_sequence is not None:
                    steps, step_index, start_time = servo_sequence
                    pos, duration = steps[step_index]
                    pwm.set_servo_pulsewidth(servo_pin, pos)

                    if current_time - start_time >= duration:
                        step_index += 1
                        if step_index >= len(steps):
                            servo_sequence = None
                        else:
                            servo_sequence = (steps, step_index, current_time)
                    color_detected = red_label if red_area > green_area else green_label
                else:
                    color_detected = "None"
            else:
                color_detected = "None"
            print("Detected:", color_detected)
            cv2.imshow("Color Detection", frame)
            
            
        else:
            if distance_cm < 30 and orange_sequence is None:
                pwm.set_servo_pulsewidth(servo_pin, 1500)
                left_dist = leftultrasonic.distance * 100
                right_dist = rightultrasonic.distance * 100
                if left_dist > right_dist and left_dist > 20:
                    pwm.set_servo_pulsewidth(servo_pin, 1000)
                elif right_dist > left_dist and right_dist > 20:
                    pwm.set_servo_pulsewidth(servo_pin, 2000)
                else:
                    pwm.set_servo_pulsewidth(servo_pin, 1500)
                    motorSpeed(0)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            motorSpeed(0)
            break

finally:
    motorSpeed(0)
    pwm.set_servo_pulsewidth(servo_pin, 0)
    cv2.destroyAllWindows()
