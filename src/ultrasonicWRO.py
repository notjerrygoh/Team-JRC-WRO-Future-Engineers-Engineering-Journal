from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

factory = PiGPIOFactory()

leftultrasonic = DistanceSensor(echo=17, trigger=4, max_distance=4, pin_factory=factory) 
rightultrasonic = DistanceSensor(echo=27, trigger=22, max_distance=4, pin_factory=factory) 
frontultrasonic = DistanceSensor(echo=5, trigger=6, max_distance=4, pin_factory=factory) 
backultrasonic = DistanceSensor(echo=19, trigger=26, max_distance=4, pin_factory=factory) 
fulleftultrasonic = DistanceSensor(echo=20, trigger=21, max_distance=4, pin_factory=factory)
fullrightultrasonic = DistanceSensor(echo=25, trigger=7, max_distance=4, pin_factory=factory)
frame = 0

while True:
    print(f"Left:  {leftultrasonic.distance:.2f} m", "#" * int(leftultrasonic.distance * 20), "\033[K")
    print(f"Right: {rightultrasonic.distance:.2f} m", "#" * int(rightultrasonic.distance * 20), "\033[K")
    print(f"Front: {frontultrasonic.distance:.2f} m", "#" * int(frontultrasonic.distance * 20), "\033[K")
    print(f"Back:  {backultrasonic.distance:.2f} m", "#" * int(backultrasonic.distance * 20), "\033[K")
    print(f"Full Left: {fulleftultrasonic.distance:.2f} m", "#" * int(fulleftultrasonic.distance * 20), "\033[K")
    print(f"Full Right: {fullrightultrasonic.distance:.2f} m", "#" * int(fullrightultrasonic.distance * 20), "\033[K")
    print("frame %d" % frame)

    frame += 1
    sleep(0.25)
    print("\033[5A\r", end="")
    print("\033[2J", end="")

