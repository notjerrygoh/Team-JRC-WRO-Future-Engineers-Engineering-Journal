# Run sudo pigpiod in terminal before running code 

import cv2
import numpy as np
from time import sleep
import pigpio
from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from picamera2 import Picamera2

# --- Setup PiGPIO ---
pi = pigpio.pi()
pwm = pigpio.pi() 
factory = PiGPIOFactory()

# --- Wall choice ---
wall_side = "left"  # Change to "right" to align along right wall

# --- Ultrasonic sensors ---
leftultrasonic = DistanceSensor(echo=27, trigger=22, max_distance=4, pin_factory=factory)
rightultrasonic = DistanceSensor(echo=17, trigger=4, max_distance=4, pin_factory=factory)
frontultrasonic = DistanceSensor(echo=5, trigger=6, max_distance=4, pin_factory=factory)
backultrasonic = DistanceSensor(echo=16, trigger=26, max_distance=4, pin_factory=factory)

if wall_side == "left":
    wall_sensor = leftultrasonic
else:
    wall_sensor = rightultrasonic

# --- Servo setup ---
servo_pin = 16
pwm.set_mode(servo_pin, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo_pin, 50)
pwm.set_servo_pulsewidth(servo_pin, 1500)  # neutral

def setServo(pw):
    pwm.set_servo_pulsewidth(servo_pin, pw)

# --- Motors ---
M1A = 23
M1B = 24
pi.set_mode(M1A, pigpio.OUTPUT)
pi.set_mode(M1B, pigpio.OUTPUT)

def motorSpeed(speed):
    speed *= 2 
    if speed > 0:
        pi.set_PWM_dutycycle(M1A, speed)
        pi.set_PWM_dutycycle(M1B, 0)
    elif speed < 0:
        pi.set_PWM_dutycycle(M1A, 0)
        pi.set_PWM_dutycycle(M1B, abs(speed))
    else:
        pi.set_PWM_dutycycle(M1A, 0)
        pi.set_PWM_dutycycle(M1B, 0)

# --- Camera setup ---
cam = Picamera2()
cam.configure(cam.create_preview_configuration({"size": (640, 360)}))
cam.start()

# --- Pink block detection ---
def detect_pink(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_pink = np.array([140, 100, 100], np.uint8)
    upper_pink = np.array([170, 255, 255], np.uint8)
    mask = cv2.inRange(hsv, lower_pink, upper_pink)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area > 500:
            x, y, w, h = cv2.boundingRect(largest)
            center_x = x + w // 2
            return True, center_x, area
    return False, None, 0

# --- Step 1: Approach pink zone ---
def approach_pink():
    print("Approaching pink blocks")
    while True:
        frame = cv2.cvtColor(cam.capture_array(), cv2.COLOR_RGB2BGR)
        detected, center_x, area = detect_pink(frame)
        distance_cm = frontultrasonic.distance * 100

        if detected:
            print(f"[PINK] Area={area:.1f}, dist={distance_cm:.1f} cm")
            if distance_cm > 10:
                motorSpeed(50)
                setServo(1500)
            else:
                motorSpeed(0)
                print("[PINK] Stopped ~10 cm before pink zone")
                return
        else:
            motorSpeed(40)
            setServo(1500)

# --- Step 2: Align parallel along wall ---
def align_parallel(tolerance=1.0, target_distance=10.0):
    print("Aligning parallel to wall")
    while True:
        wall_dist = wall_sensor.distance * 100
        error = wall_dist - target_distance

        if abs(error) <= tolerance:
            motorSpeed(0)
            setServo(1500)
            print("Robot aligned parallel to wall")
            break

        # Determine servo adjustment based on wall side
        if wall_side == "left":
            servo_pw = 1500 + int(error * 50)  # positive error -> steer right
        else:
            servo_pw = 1500 - int(error * 50)  # positive error -> steer left

        servo_pw = max(1400, min(1600, servo_pw))
        setServo(servo_pw)
        motorSpeed(30)
        sleep(0.1)
        motorSpeed(0)

# --- Step 3: Overshoot ---
def overshoot():
    print("Overshooting")
    setServo(1500)
    front_target = -0.05
    while frontultrasonic.distance > front_target:
        motorSpeed(40)
    motorSpeed(0)
    print("Done")

# --- Step 4: Reverse into slot ---
def reverse_into_slot():
    print("Reversing into slot")
    target_wall_distance = 0.05  # desired distance from wall
    setServo(1500)
    while (backultrasonic.distance * 100) > 5:  # stop x cm before rear pink block
        motorSpeed(-30)
        # Smooth servo adjustment
        wall_dist = wall_sensor.distance
        error = wall_dist - target_wall_distance
        if wall_side == "left":
            servo_pw = 1500 + int(error * 50)
        else:
            servo_pw = 1500 - int(error * 50)
        servo_pw = max(1400, min(1600, servo_pw))
        setServo(servo_pw)
    motorSpeed(0)
    setServo(1500)
    print("Done. Robot parked and aligned.")

# --- MAIN ---
try:
    approach_pink()
    align_parallel()
    overshoot()
    reverse_into_slot()
finally:
    motorSpeed(0)
    setServo(0)
    cam.stop()
    cv2.destroyAllWindows()
