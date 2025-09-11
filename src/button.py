from gpiozero import Button

button = Button(13, bounce_time=0.2, pull_up=True)
last_click = 0

def wait_for_start():
    print("Waiting...")
    button.wait_for_press()
    print("Robot Started!")

wait_for_start()
