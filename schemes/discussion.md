# Raspberry Pi
We chose to use the Raspberry Pi as the central controller for our robot because it can be a microcontroller and a microprocessor at the same time. This allows us to conserve space on the robot as we do not need an additional Arduino to control the various sensors on the robot and it also makes it easier for us to troubleshoot if there are any issues during testing as we are only dealing with one system instead of 2. The Raspberry Pi is equipped with a Pi Camera slot, which helps in the computer vision of the task. On top of that, the Raspberry Pi also has GPIO (General Purpose Input Output) pins to connect different periphrals for different applications. In this case, we will be using the GPIO pins to connect hardware such as the ultrasonic sensors, servos and DC Motors.

## Ultrasonic Sensors
For the ultrasonic sensors, we will define the following pins:
Front ultrasonic sensors (Echo: 27, Trigger: 22)
Back ultrasonic sensors (Echo: 19, Trigger: 26)
Left ultrasonic sensors (Echo: 17, Trigger: 4)
Right ultrasonic sensors (Echo: 5, Trigger: 6)

The ultrasonic sensors will then detect and calculate the distances between itself and the object and produce the calculated distance as output. This value is then read through the pi by gpiozero library with the pigpio backend.

## Servo
Using the distances detected by the ultrasonic sensors, the Pi will then activate the servo connected to GPIO pin 16 of the Raspberry Pi. The servo will be driven by PWM (Pulse-width modulation) to move at the different angles to evade the obstacles detected by thu ultrasonic sensors. When the Pi Camera detects a certain colour, it will also activate the servo motor via GPIO pin 16 to move in the different directions.

## DC Motors
The DC Motors are connected to the Raspberry Pi via a motor driver. The motor driver is there to help regulate the motor's speed, direction to ensure reliable and efficient operation. The motor driver requires 2 pins, therefore, we will be using 2 pins:
M1A: 23
M1B: 24

Both M1A and M1B are controlled via PWM duty cycles, allowing the robot to move forward, backward, or stop with varying speeds.

## Raspberry Pi
The PiCamera is connected physically to the Picamera slot in the Raspberry Pi, accessed through the Picamera2 library, captures live video frames, which are processed with OpenCV to detect colors: orange lines for lap counting, pink for parking zones, and red/green the coloured blocks. Each detection routine converts the frame to HSV, applies color thresholds, cleans up masks, and uses contours to decide whether a target color is present and how the robot should respond.

All these feed into the main loop: the Raspberry Pi continuously reads camera frames and ultrasonic distances, decides what situation the robot is in (lap crossing, obstacle detection or parking), and then sends PWM commands to the motors and servo to execute the correct action. 
