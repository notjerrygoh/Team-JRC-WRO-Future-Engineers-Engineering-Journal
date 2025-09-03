# Obstacle Management

## Ultrasonic Sensor
The ultrasonic sensor will send out pulses of ultrasonic waves when power is supplied to the ultrasonic sensor and record the time when it sends the ultrasonic waves. When the ultrasonic waves hit an object, it reflects towards the ultrasonic sensor, and the ultrasonic sensor will then receive the pulses of ultrasonic waves and record the time when it received the ultrasonic waves. Then, the program will calculate the total time taken for the ultrasonic waves to travel from the vehicle's ultrasonic sensor to the object and back using the formula taken from the Power and Sense Management Section:
                                                  
                                                   d = t * 0.034 / 2
    where, 

      d: distance between the object and the vehicle's ultrasonic sensor (in metres)
      
      t: time taken from the time when the ultrasonic sends out pulses of ultrasonic waves from the echo pin and from the
         time that the ultrasonic receives the pulses from the trigger pin (in seconds)
         
This formula will determine an approximate distance of the vehicle from the object. Within the code, there is a certain threshold value, which is the maximum distance that the vehicle can be from the object. If the distance between the vehicle and the object is less than this threshold value, then the vehicle will steer to the left or to the right depending on the colour of the block.

## Pi Camera
The Pi Camera is used as the main navigation tool of the robot. The robot will use the pi camera to detect the colour of the blocks and the lines to decide which direction to move in by turning the servo accordingly. It does this by first initialising the camera in 640 x 360 resolution. This is to help the pi process every frame captrured by the camera for classification quickly and effectvely. Every frame is captured in an numPy array and each frame is inverted for processing as the default configuration for the Pi camera is the frame not being upright, which can affect data processsing. We will be using OpenCV to help detect the colours in each frame, which is why we convert each frame from RGB to BGR as OpenCV only accepts data in BGR format for processing. 

After detecting each frame, we will convert them into the HSV format to separate colour information (Hue) from intensity (Value) and saturation (Saturation) and make recognising objects much easier in varying lighting conditions. This is to help our robot be able to prepare for the change in lighting conditions on competition day. Once done, we will define the HSV range for each colour we are detecting (E.g Pink, Red, Green), with a lower and upper limit so that we can detect a range of colours in the case that our shade of selectred colours when doing testing is not the same shade during the competition. These values are found through online research as well as during testing. Using the upper and lower limits, we will then create a mask for each colour, highlighting only the pixels in that range. These masks created will then be cleaned with dilation and analyzed using contour detection to find the size and position of the colored regions and identify the colours present in the frame. This will help the robot to perform the relevant logic based on the colours detected. Examples include pink indicates the parking zone and helps align the robot to drive towards the parking zone, while red and green control servo sequences to allow the robot to avoid the coloured blocks.
