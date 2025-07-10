from turtle import left
import numpy as np
import cv2
import Rpi.GPIO as GPIO
from gpiozero import DistanceSensor
from time import sleep


#Setting up servo motor
GPIO.setmode(GPIO.BOARD)
GPIO.setup(03, GPIO.OUT) 
PWM=GPIO.PWM(03, 50)
PWM.start(0)

#Turning the servo motor
def Angle(angle):
	duty = angle / 18 + 2
	GPIO.output(03, True)
	PWM.ChangeDutyCycle(duty)
	sleep(1)
	GPIO.output(03, False)
	PWM.ChangeDutyCycle(0)



#Setting up ultrasonic sensors
leftultrasonic = DistanceSensor(echo=17, trigger=4, threshold_distance=0.5) 
rightultrasonic = DistanceSensor(echo=17, trigger=4, threshold_distance=0.5) 
backultrasonic = DistanceSensor(echo=17, trigger=4, threshold_distance=0.5) 
frontultrasonic = DistanceSensor(echo=17, trigger=4, threshold_distance=0.5) 


# turn on cam
webcam = cv2.VideoCapture(0)


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

    # red 
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, mask=red_mask)

    # green 
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame, mask=green_mask)

    #pink
    pink_mask = cv2.dilate(pink_mask, kernal)
    res_pink = cv2.bitwise_and(imageFrame, imageFrame, mask=pink_mask)

    # Creating contour for red
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (0, 0, 255), 2)

            cv2.putText(imageFrame, "Red Block", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255))

    # Creating contour for green
    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (0, 255, 0), 2)

            cv2.putText(imageFrame, "Green Block", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0, 255, 0))
    
    # Creating contour for pink
    contours, _ = cv2.findContours(pink_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 300:
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (255, 0, 255), 2)
            cv2.putText(imageFrame, "Pink Wall", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 255), 2)

    # final run
    cv2.imshow("Color Detection", imageFrame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        webcam.release()
        cv2.destroyAllWindows()
        break