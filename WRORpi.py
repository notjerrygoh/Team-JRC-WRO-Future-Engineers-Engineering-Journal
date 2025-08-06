import numpy as np
import cv2
from gpiozero import DistanceSensor 
from gpiozero import LED, Button, Buzzer
import pigpio
import RPi.GPIO as GPIO
from time import sleep

#Ultrasonic sensors
leftultrasonic = DistanceSensor(echo = 17, trigger = 4, threshold_distance = 0.3)
rightultrasonic = DistanceSensor(echo = 27, trigger = 22, threshold_distance = 0.3)
frontultrasonic = DistanceSensor(echo = 5, trigger = 6, threshold_distance = 0.3)
backultrasonic = DistanceSensor(echo = 13, trigger = 26, threshold_distance = 0.3)


#Servo motors
SERVO_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz PWM signal
pwm.start(0)   

#DC Motors
M1A = 1
M1B = 7
M2A = 12
M2B = 16

pi.set_mode(M1A, pigpio.OUTPUT)
pi.set_mode(M1B, pigpio.OUTPUT)
pi.set_mode(M2A, pigpio.OUTPUT)
pi.set_mode(M2B, pigpio.OUTPUT)

def start_motors():
    pi.write(M1A, 1)
    pi.write(M2A, 1)
    pi.set_PWM_dutycycle(M1B, 200)
    pi.set_PWM_dutycycle(M2B, 200)

def stop_motors():
    pi.set_PWM_dutycycle(M1B, 0)
    pi.set_PWM_dutycycle(M2B, 0)

# Checks colour
def check_colour(imageFrame):
    while True:
        ret, imageFrame = webcam.read()

        # Convert BGR to HSV colorspace
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

        # Range of red
        red_lower = np.array([136, 87, 111], np.uint8)
        red_upper = np.array([180, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

        #Range of green
        green_lower = np.array([25, 52, 72], np.uint8)
        green_upper = np.array([102, 255, 255], np.uint8)
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

        #Range of pink
        pink_lower = np.array([160, 100, 100], np.uint8)
        pink_upper = np.array([180, 255, 255], np.uint8)
        pink_mask = cv2.inRange(hsvFrame, pink_lower, pink_upper)


        # to detect only that particular color
        kernal = np.ones((5, 5), "uint8")
        red_mask = cv2.dilate(red_mask, kernal)
        green_mask = cv2.dilate(green_mask, kernal)
        pink_mask = cv2.dilate(pink_mask, kernal)

        # Check contours
        def detect_color(mask, color_name, color_bgr):
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 300:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(imageFrame, (x, y), (x + w, y + h), color_bgr, 2)
                    cv2.putText(imageFrame, f"{color_name} Block", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, color_bgr, 2)
                    return True
            return False
    
        red_found = detect_color(red_mask, "Red", (0, 0, 255))
        green_found = detect_color(green_mask, "Green", (0, 255, 0))
        pink_found = detect_color(pink_mask, "Pink", (255, 0, 255))
        red_found = '1'
        green_found = '2'

        return red_found or green_found or pink_found

def set_angle(angle):
    duty = 2 + (angle / 18)
    GPIO.output(SERVO_PIN, True)
    pwm.ChangeDutyCycle(duty)
    sleep(0.3)
    GPIO.output(SERVO_PIN, False)
    pwm.ChangeDutyCycle(0)

# turns on cam
webcam = cv2.VideoCapture(0)


while True:
    start_motors()
    if frontultrasonic.in_range:

        ret, imageFrame = webcam.read()
        if not ret:
            continue

        check_colour(imageFrame)
        if check_colour(imageFrame) == '1':
            set_angle(0)
        elif check_colour(imageFrame) == '2':
            set_angle(180)
        else:
            set_angle(90) 

        # Quit with 'q'
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    else:
        # No block detected close enough — don't process camera
        sleep(0.1)
