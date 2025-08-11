import numpy as np
import cv2
from gpiozero import DistanceSensor 
from gpiozero import LED, Button, Buzzer
import pigpio
import RPi.GPIO as GPIO
from time import sleep


# Ultrasonic sensors
leftultrasonic = DistanceSensor(echo=17, trigger=4, threshold_distance=0.3)
rightultrasonic = DistanceSensor(echo=27, trigger=22, threshold_distance=0.3)
frontultrasonic = DistanceSensor(echo=5, trigger=6, threshold_distance=0.3)
backultrasonic = DistanceSensor(echo=13, trigger=26, threshold_distance=0.3)

# Servo motors
servo_pin = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz PWM signal
pwm.start(0)   

# DC Motors
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
    # Convert BGR to HSV colorspace
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    # Red ranges
    red_lower1 = np.array([0, 120, 70], np.uint8)
    red_upper1 = np.array([10, 255, 255], np.uint8)
    red_mask1 = cv2.inRange(hsvFrame, red_lower1, red_upper1)

    red_lower2 = np.array([170, 120, 70], np.uint8)
    red_upper2 = np.array([180, 255, 255], np.uint8)
    red_mask2 = cv2.inRange(hsvFrame, red_lower2, red_upper2)

    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    # Green range
    green_lower = np.array([25, 52, 72], np.uint8)
    green_upper = np.array([102, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    # Pink range 
    pink_lower = np.array([145, 80, 80], np.uint8)
    pink_upper = np.array([165, 255, 255], np.uint8)
    pink_mask = cv2.inRange(hsvFrame, pink_lower, pink_upper)

    kernel = np.ones((5, 5), "uint8")
    red_mask = cv2.dilate(red_mask, kernel)
    green_mask = cv2.dilate(green_mask, kernel)
    pink_mask = cv2.dilate(pink_mask, kernel)

    def detect_color(mask, color_name, color_bgr, return_val):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 300:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(imageFrame, (x, y), (x + w, y + h), color_bgr, 2)
                cv2.putText(imageFrame, f"{color_name} Block", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, color_bgr, 2)
                return return_val
        return None

    red_found = detect_color(red_mask, "Red", (0, 0, 255), '1')
    if red_found:
        return red_found
    green_found = detect_color(green_mask, "Green", (0, 255, 0), '2')
    if green_found:
        return green_found
    pink_found = detect_color(pink_mask, "Pink", (255, 0, 255), '3')
    if pink_found:
        return pink_found

    return None

def set_angle(angle):
    duty = 2 + (angle / 18)
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    sleep(0.3)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

# turns on cam
webcam = cv2.VideoCapture(0)

while True:
    start_motors()
    if frontultrasonic.in_range:
        ret, imageFrame = webcam.read()
        if not ret:
            continue

        detected = check_colour(imageFrame)
        if detected == '1':
            set_angle(0)
        elif detected == '2':
            set_angle(180)
        else:
            set_angle(90)
        """
        elif detected == '3':
            set_angle(90)
        """
       

        # Show the frame with rectangles
        cv2.imshow("Color Detection", imageFrame)

        # Quit with 'q'
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    else:
        # No block detected close enough — don't process camera
        sleep(0.1)

# Cleanup
webcam.release()
cv2.destroyAllWindows()
pwm.stop()
pi.stop()
GPIO.cleanup()
