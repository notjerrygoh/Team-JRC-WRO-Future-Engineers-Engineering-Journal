from gpiozero import Button
from time import sleep, time

button = Button(13, pull_up=True)  # enable internal pull-up

last_click = 0

def pressed():
    global last_click
    now = time()
    if now - last_click < 1:  # double press within 1 sec
        print("⏹ Double click detected → stop")
    else:
        print("▶ Single click → start")
    last_click = now

button.when_pressed = pressed

print("Waiting for button presses...")
while True:
    sleep(0.1)
