# Pseudocode
IMPORT Numerical Python library (Numpy)
IMPORT computer vision library (OpenCV)
IMPORT DistanceSensor class from GPIO Zero library for ultrasonic sensors
IMPORT LED, Button, and Buzzer classes from GPIO Zero library
IMPORT pigpio library for GPIO control
IMPORT RPi.GPIO library for GPIO setup
IMPORT sleep function from time module for delays


leftultrasonic = Reads ultrasonic sensor distance for the ultrasonic sensor on the left with GPIO pin 17 as echo and GPIO pin 4 as echo and a threshold distance of 0.3m
rightultrasonic = Reads ultrasonic sensor distance for the ultrasonic sensor on the left with GPIO pin 27 as echo and GPIO pin 22 as echo and a threshold distance of 0.3m
frontultrasonic = Reads ultrasonic sensor distance for the ultrasonic sensor on the left with GPIO pin 5 as echo and GPIO pin 6 as echo and a threshold distance of 0.3m
backultrasonic = Reads ultrasonic sensor distance for the ultrasonic sensor on the left with GPIO pin 13 as echo and GPIO pin 26 as echo and a threshold distance of 0.3m

servo_pin = GPIO pin 18
SET GPIO pin 18 connected to the servo as output pin
SET PWM signal on servo_pin with a frequency of 50 Hz 
SET PWM with a duty cycle of 0%

M1A = GPIO pin 1
M1B = GPIO pin 7
M2A = GPIO pin 12 
M2B = GPIO pin 16

SET M1A as output pin using pigpio
SET M1B as output pin using pigpio
SET M2A as output pin using pigpio
SET M2B as output pin using pigpio

FUNCTION start_motors:
  SET M1A as HIGH
  SET M2A as HIGH
  SET PWM Duty cycle of M1B to 200 
  SET PWN DUty Cycle of M2B to 200
ENDFUNCTION

FUNCTION stop_motors:
  SET M1B as LOW
  SET M2B as LOW
ENDFUNCTION

FUNCTION check_colours:
  WHILE TRUE
    Capture an frame from the webcam
    Convert the frame from BGR to HSV colour format (Makes it easier to detect colours HSV (Hue, Saturation, Value)
    DEFINE lower and upper limits for detecting red, green and pink colours
    CREATE masks that isolates areas that match the ranges n the lower and upper limits of the red, green and pink colours 
    CREATE a 5x5 matrix for image dilation to make it easier to analyse images.
    APPLY dilation to each mask to reduce noise and enhance object shapes
    FUNCTION detect_color(mask, color_name, color_bgr):
      FIND contours in the colour mask
      FOR contour in contours:
        area = calculate area of the contours
        IF area > 300
          GET the rectangle boundary of the object (x, y, width, height)
          DRAW a rectangle around the object on the image
          LABEL the object with its color name
          RETURN True 
      RETURN False
      IF red object detected:
        red_found = '1'
      IF green object detected:
        green_found = '2'
  RETURN red_found, green_found, pink_found
ENDFUNCTION

FUNCTION set_angle(angle):
  duty = 2 + (angle / 18)
  

    
    




