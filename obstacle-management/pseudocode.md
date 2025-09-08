START

FROM gpiozero IMPORT DistanceSensor
FROM gpiozero.pins.pigpio IMPORT PiGPIOFactory
FROM time IMPORT time
IMPORT pigpio
IMPORT numpy
IMPORT cv2
FROM picamera2 IMPORT Picamera2

INITIALISE pigpio library and assign to "pi"
INITIALISE GPIO pin factory and assign to "factory"

frontultrasonic = Reads ultrasonic sensor distance for the ultrasonic sensor on the left with GPIO pin 27 as echo and GPIO pin 22 as echo
leftultrasonic = Reads ultrasonic sensor distance for the ultrasonic sensor on the left with GPIO pin 17 as echo and GPIO pin 4 as echo
rightultrasonic = Reads ultrasonic sensor distance for the ultrasonic sensor on the left with GPIO pin 5 as echo and GPIO pin 6 as echo
backultrasonic = Reads ultrasonic sensor distance for the ultrasonic sensor on the left with GPIO pin 19 as echo and GPIO pin 26 as echo

servo_pin = GPIO pin 16
SET GPIO pin 18 connected to the servo as output pin
SET PWM signal on servo_pin with a frequency of 50 Hz 
SET PWM of servo_pin to 1500

INITIALISE Picamera and a preview with a resolution of 640 x 360

M1A = GPIO pin 23
M1B = GPIO pin 24
SET M1A as output pin using pigpio
SET M1B as output pin using pigpio

FUNCTION detect_orange(frame, on_orange_line, lap_count):
  CAPTURE a frame from the webcam and convert the frame from BGR to HSV (Hue, Saturation, Value) colour format (Makes it easier to detect colours)
  DEFINE lower/upper HSV limits for orange:
    orange_upper = H: 5, S: 150, V: 150
    orange_lower = H: 20, S: 255, V: 255
  orange_mask = Creates a mask that contains only pixels within the orange range (orange_upper, orange_lower)
  roi = Bottom 60 pixels of the orange_mask given the height and width
  orange_pixels = The number of non-zero pixels in roi identified
  
  crossed = False
  IF orange_pixels > 1000:
    IF NOT on_orange_line:
      lap_count += 1
      crossed = True
      on_orange_line = True 
    ELSE:
      on_orange_line = False
    ENDIF
  ENDIF

  contours, _ = Find all of the contours in orange_mask
  IF contours:
    largest = The contour with the largest area
    IF largest > 300:
      x, y, w, h = Get bounding rectangle of largest contour as (x, y, w, h)
      DRAW a rectangle on the frame at (x, y, x + w, y + h) in orange color
      ENDIF
  ENDIF

RETURN lap_count, on_orange_line, crossed
      
FUNCTION traffic_lights(frame):
  CAPTURE a frame from the webcam and convert the frame from BGR to HSV (Hue, Saturation, Value) colour format (Makes it     easier to detect colours)
  DEFINE lower/upper HSV limits for red:
    red_lower1 = H: 0, S: 150, V: 50
    red_upper1 = H: 5, S: 255, V: 255
    red_lower2 = H: 172, S: 150, V: 100
    red_upper2 = H: 180, S: 255, V: 255
  red_mask = Creates a mask that contains only pixels from both ranges (red_lower1, red_upper1) and (red_lower2, red_upper2)

  DEFINE lower/upper HSV limits for red:
    green_lower = H: 35, S: 50, V: 50
    green_upper = H: 85, S: 255, V: 255
  green_mask = Creates a mask that contains only pixels within the green range (green_lower, green_upper)

  kernel = Create a 5x5 matrix
  red_mask = Dilates the red_mask 
  green_mask = Dilates the green_mask

  FUNCTION detect_and_label(mask, color_bgr, label):
    contours, _ = Find all of the contours in mask
    IF NOT contours:
      RETURN 0, None
      ENDIF
    largest = The contour with the largest area
    area = The largest area
    IF 30000 < area < 60000:  
      x, y, w, h = Get bounding rectangle of largest contour as (x, y, w, h)
      DRAW a rectangle on the frame at (x, y, x + w, y + h) in orange color
      RETURN area, label
      ENDIF
    RETURN 0, None
  
  red_area, red_label = detect_and_label(red_mask, (0, 0, 255), "Red")
  green_area, green_label = detect_and_label(green_mask, (0, 255, 0), "Green")
  RETURN red_area, green_area, red_label, green_label

FUNCTION motorSpeed(speed):
  IF speed > 0:
    PWM Dutycycle of M1A = speed
    PWM Dutycycle of M1B = 0
  ELSEIF speed < 0:
    PWM Dutycycle of M1A = 0
    PWM Dutycycle of M1B = absolute value of speed
  ELSE:
    PWM Dutycycle of M1A = 0
    PWM Dutycycle of M1B = 0
  ENDIF

lap_count = 0
on_orange_line = False
orange_sequence = None  
servo_sequence = None  
stable_counter = 0 

WHILE True:
  frame = Capture frame from the PiCamera while converting the frame from RGB to BGR and also inverting the frame
  distance_cm = Distance detected by the frontultrasonic * 100
  current_time = time()
  IF lap_count < 12:
    lap_count, on_orange_line, crossed = detect_orange(frame, on_orange_line, lap_count)
    motorSpeed(100)
    IF crossed and orange_sequence is None:
      orange_sequence = ([(1500, 1.5), (2000, 1), (1500, 0)], steps = 0, current_time) (Servo PWM value, duration to hold)
      OUTPUT lap_count
    ENDIF
    IF orange_sequence is not None:
      steps, step_index, start_time = orange_sequence
      pos, duration = steps[steps_index]
      SET servo PWM value on servo_pin to be pos
      IF current_time - start_time >= duration:
        step_index += 1
        IF step_index >= len(steps):
          orange_sequence = None
        ELSE:
          orange_sequence = (steps, step_index, current_time)
        ENDIF
      ENDIF
    ENDIF
    IF orange_sequence is None:
      red_area, green_area, red_label, green_label = traffic_lights(frame)
      IF red_area > green_area or green_area > red_area:
        stable_counter += 1
      ELSE:
        stable_counter = 0
    IF servo_sequence is None and stable_counter >= 3:  
      IF red_area > green_area:
        servo_sequence = ([(2000,1), (1500,1), (1000,1), (1500,0)], 0, current_time + 0.1)
        color_detected = red_label
      ELSEIF green_area > red_area:
        servo_sequence = ([(1000,1), (1500,1), (2000,1), (1500,0)], 0, current_time + 0.1)
        color_detected = green_label
      ELSE:
        SET servo PWM value on servo_pin to be 1500
        color_detected = "None"
      ENDIF
    ELSEIF servo_sequence is not None:
        steps, step_index, start_time = servo_sequence
        pos, duration = steps[step_index]
        SET servo PWM value on servo_pin to be pos
        IF current_time - start_time >= duration:
          step_index += 1
          IF step_index >= len(steps):
              servo_sequence = None
          ELSE:
              servo_sequence = (steps, step_index, current_time)
          ENDIF
      color_detected = red_label IF red_area > green_area else green_label
    ELSE:
      color_detected = "None"
    ENDIF
    OUTPUT colour_detected
    cv2.imshow("Color Detection", frame)
 
 ELSE: #Parking logic here
  IF distance_cm < 30 and orange_sequence is None:
      SET servo PWM value on servo_pin to be 1500
      left_dist = leftultrasonic.distance * 100
      right_dist = rightultrasonic.distance * 100
      if left_dist > right_dist and left_dist > 20:
          SET servo PWM value on servo_pin to be 1000
      elif right_dist > left_dist and right_dist > 20:
          SET servo PWM value on servo_pin to be 2000
      else:
          SET servo PWM value on servo_pin to be 1500
          motorSpeed(0)

  IF cv2.waitKey(10) & 0xFF == ord('q'):
    motorSpeed(0)
    ENDWHILE

motorSpeed(0)
pwm.set_servo_pulsewidth(servo_pin, 0)
cv2.destroyAllWindows()
END
