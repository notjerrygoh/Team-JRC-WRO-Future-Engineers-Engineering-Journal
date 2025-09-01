Team JRC's Engineering Journal
====

Hello! We are team JRC, consisting of Jeremiah, Colin and Rey from Ngee Ann Polytechnic. We are participating in the World Robot Olympiad (WRO) Future Engineers competition in the season 2025. This repository contains engineering materials of the automated vehicle's model, as well as our engineering journal, where we document our experiences and the engineering design process even as we design and build our robot.

## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `mobility-management` contains the discussion for mobility management on how the motors are selected and implemented into our vehicle, as well as the vehicle chassis design selection and how the components are mounted onto the vehicle.
* `power-and-sense-management` contains the discussion for power and sense management on how the different power supply sources, as well as the sensors to detect objects are selected and implemented into our vehicle.
* `obstacle-management` contains the discussion for obstacle management on the vehicle's logic when it comes to avoiding obstacles and parking, and contains files of the pseudocode.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

## Introduction
The code consists of the following segments:
1. Colour Detection
2. Servo Movement
3. DC motor movement
4. Object detection
5. Line detection
6. Parking

### Basic functions
<u>Colour detection</u>
The robot will detect the colours by the Pi camera that is attached to the Raspberry Pi. It does this by first giving it a predefined upper and lower limit of values in HSV (Hue, Saturation, Value) so that the Pi camera can use these values to create a range and eventually a mask to help detect the colours when given a frame captured.

<u>Servo movement</u>
The servo will move by using PWM by sending a signal through the connected GPIO pin. The servo, given the PWM value, would turn in either the clockwise and anticlockwise direction from the starting position.

<u>DC motor Movement</u>
The DC Motor uses a motor driver to drive the DC Motor with an external power supply (9V battery) as the 5V from the Raspberry Pi is not enough to sustain it. We will connect the motor driver to 2 pins, 23 and 24. GPIO pins 23 and 24 connect to M1A and M1B respectively, allowing the robot to move forwards and backwards given values entered into the code.

### Applications
<u>Object Detection</u>
This applications combines the colour detection with the servo movement. When the Pi Camera approaches the red or green block on the map, it will detect the colour of the incoming block by the Pi Camera and the servo would then perform a series of turns in order to avoid the obstacle by using PWM values. One such example is as follows:
If green block detected: 1000 (Turn Left) --> 1500 (Stabilise) --> 2000 (Turn Right) ---> 1500 (Stabilise)

This helps the robot to avoid bumping into the obstacle in front all while moving forward

<u>Line Detection</u>
This application combines the colour detection as well as the servo movement. When the Pi Camera approaches an orange line, it will detect the orange line and then perform a series of maneuvres to turn. 
If orange line detected: Continue moving for 2 seconds (This is to ensure that the robot crosses the line) --> 1000 (for 2 seconds) --> 1500 (resume original position)
