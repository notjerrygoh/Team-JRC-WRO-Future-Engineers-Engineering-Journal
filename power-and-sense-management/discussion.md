# Power and sense management

## Power management
### Powerbank
For the power source of the robot, we have opted to use a power bank to power the Raspberry Pi 4, by using a USB-C cable to connect the power bank to the Raspberry Pi 4. This is because it has enough power to supply the Raspberry Pi 4 (which requires 15W (15 watts) for the Raspberry Pi 4 to run), unlike a battery, which does not supply enough power for the Raspberry Pi 4 to run.

### 9V battery
After much consideration, we also decided to add a 9V (9-volt) battery to supply the H-Bridge Motor Driver, as the Raspberry Pi 4 alone consumes lots of power from the power bank, and the power bank might not have enough power left to operate the H-Bridge Motor Driver, since it is connected in series. Hence, we decided to use a 9V battery to supply power to the Motor Driver for its operation, so that it will be able to drive the motors of the vehicle.

## Sense Management
### Ultrasonic Sensors
To complete the challenge, we have selected the HC-SR04 Ultrasonic sensor to detect obstacles that are around the vehicle while it is running. The ultrasonic sensors are used because they are inexpensive compared to other types of sensors, and because we can easily determine the distance from the vehicle to the object using the following formula:
                                      
                                                   d = t * 0.034 / 2
    where, 

      d: distance between the object and the vehicle's ultrasonic sensor (in metres)
      
      t: time taken from the time when the ultrasonic sends out pulses of ultrasonic waves from the echo pin and from the
         time that the ultrasonic receives the pulses from the trigger pin (in seconds)

This formula will help us in determining the distance of the vehicle from the object, and is vital in ensuring that the vehicle will avoid the obstacles as it moves around the circuit, which will be further discussed in the obstacle management section.

### Pi Camera
The Pi Camera is used as the main navigation tool of the robot. The robot will use the pi camera to detect the colour of the blocks and the lines to decide which direction to move in by turning the servo accordingly. It does this by first initialising the camera in 640 x 360 resolution. This is to help the pi process every frame captured by the camera for classification quickly and effectvely. Every frame is captured in an numPy array and each frame is inverted for processing as the default configuration for the Pi camera is the frame not being upright, which can affect data processsing. We will be using OpenCV to help detect the colours in each frame, which is why we convert each frame from RGB to BGR as OpenCV only accepts data in BGR format for processing. 

After detecting each frame, we will convert them into the HSV format to separate colour information (Hue) from intensity (Value) and saturation (Saturation) and make recognising objects much easier in varying lighting conditions. This is to help our robot be able to prepare for the change in lighting conditions on competition day. Once done, we will define the HSV range for each colour we are detecting (E.g Orange and Pink), with a lower and upper limit so that we can detect a range of colours in the case that our shade of selectred colours when doing testing is not the same shade during the competition. These values are found through online research as well as during testing. Using the upper and lower limits, we will then create a mask for each colour, highlighting only the pixels in that range. These masks created will then be cleaned with dilation and analyzed using contour detection to find the size and position of the colored regions and identify the colours present in the frame. This will help the robot to perform the relevant logic based on the colours detected. Examples include orange signals a lap line and increments the lap counter, pink indicates a parking zone which helps the robot position itself for parking.
