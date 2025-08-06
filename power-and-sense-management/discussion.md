# Power and sense management

## Power management: Powerbank
For the power source of the robot, we have opted to use a power bank to power the Raspberry Pi 4, by using a USB-C cable to connect the power bank to the Raspberry Pi 4. This is because it has enough power to supply the Raspberry Pi 4 (which requires 15W (15 watts) for the Raspberry Pi 4 to run), unlike a battery, which does not supply enough power for the Raspberry Pi 4 to run.

## Power management: 9V battery
After much consideration, we also decided to add a 9V (9-volt) battery to supply the H-Bridge Motor Driver, as the Raspberry Pi 4 alone consumes lots of power from the power bank, and the power bank might not have enough power left to operate the H-Bridge Motor Driver, since it is connected in series. Hence, we decided to use a 9V battery to supply power to the Motor Driver for its operation, so that it will be able to drive the motors of the vehicle.

## Ultrasonic Sensors
To complete the challenge, we have selected the HC-SR04 Ultrasonic sensor to detect obstacles that are around the vehicle while it is running. The ultrasonic sensors are used because they are inexpensive compared to other types of sensors, and because we can easily determine the distance from the vehicle to the object using the following formula:
                                      
                                                   d = t * 0.034 / 2
    where, 

      d: distance between the object and the vehicle's ultrasonic sensor (in metres)
      
      t: time taken from the time when the ultrasonic sends out pulses of ultrasonic waves from the echo pin and from the
         time that the ultrasonic receives the pulses from the trigger pin (in seconds)

This formula will help us in determining the distance of the vehicle from the object, and is vital in ensuring that the vehicle will avoid the obstacles as it moves around the circuit, which will be further discussed in the obstacle management section.
