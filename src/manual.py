import pigpio
from time import sleep

import sys
import termios
import io

pi = pigpio.pi()
M1A = 23
M1B = 24

servo = 16

pi.set_mode(M1A, pigpio.OUTPUT)
pi.set_mode(M1B, pigpio.OUTPUT)
pi.set_mode(servo, pigpio.OUTPUT)
pi.set_PWM_frequency(servo, 50)

cspeed = 0
cdir = 0
crev = 0
cbrake = True
coffset = 300

def motorSpeed(speed):
    m1aspeed = 0
    m1bspeed = 0
    if (speed < 0):
        m1aspeed = -speed
    else:
        m1bspeed = speed

    try:
        pi.set_PWM_dutycycle(M1A, m1aspeed)
        pi.set_PWM_dutycycle(M1B, m1bspeed)
    except Exception as e:
        print(e)

def update():
    realspeed = 0 if cbrake else (-cspeed if crev else cspeed)
    motorSpeed(realspeed)
    pi.set_servo_pulsewidth(servo, 1500 + cdir * coffset)

print("current speed", cspeed)

def clamp(d, m, x):
    r = d
    if r < m:
        r = m
    if r > x:
        r = x
    return r

def handle(inp):
    global cspeed, cdir, crev, cbrake

    match inp:
        case " ":
            cbrake = not cbrake
        case "a":
            cdir -= 1
        case "d":
            cdir += 1
        case "s":
            crev = True
        case "w":
            crev = False
        case "1":
            cspeed = 0
        case "2":
            cspeed -= 10
        case "3":
            cspeed += 10
        case "4":
            cspeed = 100
        case "5":
            cspeed = 255


    cspeed = clamp(cspeed, 0, 255)
    cdir = clamp(cdir, -1, 1)
    sdir = ["left", "straight", "right"][cdir + 1]
    sbf = ["forwards", "backwards"][int(crev)]
    sbr = ["(no brake)", "(brake)"][int(cbrake)]
    print(cspeed, "spd", sdir, sbf, sbr)

    update()
            

oldattr = termios.tcgetattr(sys.stdin.fileno())
newattr = oldattr[::]
newattr[3] &= ~termios.ICANON
newattr[3] &= ~termios.ECHO
termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, newattr)


stdin = io.open(sys.stdin.fileno(), 'rb', closefd=False)

while True:
    try:
        handle(stdin.read(1).decode("utf-8"))
    except KeyboardInterrupt:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, oldattr)
        break
