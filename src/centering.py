from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from time import time, sleep
import pigpio

pi = pigpio.pi()


# Ultrasonic sensors
factory = PiGPIOFactory()
frontultrasonic = DistanceSensor(echo=5, trigger=6, max_distance=4, pin_factory=factory)
leftultrasonic = DistanceSensor(echo=17, trigger=4, max_distance=4, pin_factory=factory)
rightultrasonic = DistanceSensor(echo=27, trigger=22, max_distance=4, pin_factory=factory)
backultrasonic = DistanceSensor(echo=19, trigger=26, max_distance=4, pin_factory=factory)
turn_amt = 200
offset = -50

# Servo
servo_pin = 16
pwm = pigpio.pi()
pwm.set_mode(servo_pin, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo_pin, 50)
pwm.set_servo_pulsewidth(servo_pin, 1500 + offset)  

M1A = 23
M1B = 24
pi.set_mode(M1A, pigpio.OUTPUT)
pi.set_mode(M1B, pigpio.OUTPUT)


def motorSpeed(speed):
    if speed < 0:
        pi.set_PWM_dutycycle(M1A, abs(speed))
        pi.set_PWM_dutycycle(M1B, 0)
    elif speed > 0:
        pi.set_PWM_dutycycle(M1A, 0)
        pi.set_PWM_dutycycle(M1B, abs(speed))
    else:
        pi.set_PWM_dutycycle(M1A, 0)
        pi.set_PWM_dutycycle(M1B, 0)

frame = 0
def align_center(left_target, right_target, tolerance):
    global frame
    while True:
        left_dist = leftultrasonic.distance * 100
        right_dist = rightultrasonic.distance * 100
      
        if abs(left_dist - left_target) <= tolerance and abs(right_dist - right_target) <= tolerance:
            motorSpeed(0)
            pwm.set_servo_pulsewidth(servo_pin, 1500 + offset)  # straighten

        if left_dist < left_target:
            pwm.set_servo_pulsewidth(servo_pin, 1500 + offset + turn_amt)
            motorSpeed(60)

        elif right_dist < right_target:
            pwm.set_servo_pulsewidth(servo_pin, 1500 + offset - turn_amt)
            motorSpeed(60)

        else:
            # If both distances are bigger, just go straight and let sensors stabilize
            pwm.set_servo_pulsewidth(servo_pin, 1500 + offset)
            motorSpeed(60)

        left_dist /= 100
        right_dist /= 100
        print(f"Left:  {left_dist:.2f} m", "#" * int(left_dist * 20), "\033[K")
        print(f"Right: {right_dist:.2f} m", "#" * int(right_dist * 20), "\033[K")
        print("frame %d" % frame)

        # Small delay to avoid overshoot
        time_sleep = 0.1
        sleep(time_sleep)
        # print("\033[3A\r", end="")
        frame += 1


motorSpeed(60)
sleep(0.5)
align_center(49, 47, 5)
