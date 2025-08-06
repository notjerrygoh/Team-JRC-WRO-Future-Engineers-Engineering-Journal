# Obstacle Management

## Ultrasonic Sensor
The ultrasonic sensor will send out pulses of ultrasonic waves when power is supplied to the ultrasonic sensor and record the time when it sends the ultrasonic waves. When the ultrasonic waves hit an object, it reflects towards the ultrasonic sensor, and the ultrasonic sensor will then receive the pulses of ultrasonic waves and record the time when it received the ultrasonic waves. Then, the program will calculate the total time taken for the ultrasonic waves to travel from the vehicle's ultrasonic sensor to the object and back using the formula taken from the Power and Sense Management Section:
                                                  
                                                   d = t * 0.034 / 2
    where, 

      d: distance between the object and the vehicle's ultrasonic sensor (in metres)
      
      t: time taken from the time when the ultrasonic sends out pulses of ultrasonic waves from the echo pin and from the
         time that the ultrasonic receives the pulses from the trigger pin (in seconds)
         
This formula will determine an approximate distance of the vehicle from the object. Within the code, there is a certain threshold value, which is the maximum distance that the vehicle can be from the object. If the distance between the vehicle and the object is less than this threshold value, then the vehicle will steer to the left or to the right depending on the colour of the block.
