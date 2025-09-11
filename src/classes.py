from gpiozero import Button, DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from time import time, sleep

import pigpio
from pigpio import OUTPUT

import numpy as np
import cv2
from picamera2 import Picamera2

def clamp(d, m, x):
    r = d
    if r < m:
        r = m
    if r > x:
        r = x
    return r

class Controller:
    pi = None
    servo = 16
    ma = 23
    mb = 24

    auto_update = False

    cspeed = 0
    cdir = 0
    cturn = 300
    coffset = 0

    def __init__(self, ma_pin, mb_pin, servo_pin):
        self.ma = ma_pin
        self.mb = mb_pin
        self.servo = servo_pin

        self.pi = pigpio.pi()
        pi.set_mode(self.ma, OUTPUT)
        pi.set_mode(self.mb, OUTPUT)

        pi.set_PWM_frequency(self.servo, 50)

    def update(self):
        rspd = abs(cspeed)
        if cspeed < 0:
            self.pi.set_PWM_dutycycle(self.ma, rspd)
            self.pi.set_PWM_dutycycle(self.mb, 0)
        else:
            self.pi.set_PWM_dutycycle(self.ma, 0)
            self.pi.set_PWM_dutycycle(self.mb, rspd)

        rdr = 1500 + self.coffset + self.cdir * self.cturn
        rdr = clamp(rdr, 500, 2500)
        self.pi.set_servo_pulsewidth(self.servo, rdr)

        return self

    def speed(self, newspd: int):
        self.cspeed = newspd
        return self.update() if auto_update else self

    def dir(self, newdir: int):
        self.cdir = clamp(newdir, -1, 1)
        return self.update() if auto_update else self

    def turn(self, newtrn: int):
        self.cturn = newtrn
        return self.update() if auto_update else self

    def offset(self, newoff: int):
        self.coffset = newoff
        return self.update() if auto_update else self

class Ultrasonics:
    front = None
    left = None
    right = None
    back = None

    factory = None

    fe = 5
    ft = 6
    le = 17
    lt = 4
    re = 27
    rt = 22
    be = 19
    bt = 26

    max_dist = 4

    def __init__(self):
        pass

    def set_front(self, echo: int, trigger: int):
        self.fe = echo
        self.ft = trigger

    def set_left(self, echo: int, trigger: int):
        self.le = echo
        self.lt = trigger

    def set_right(self, echo: int, trigger: int):
        self.re = echo
        self.rt = trigger

    def set_back(self, echo: int, trigger: int):
        self.be = echo
        self.bt = trigger

    def init(self):
        self.factory = PiGPIOFactory()

        self.front = DistanceSensor(echo=self.fe, trigger=self.ft, max_distance=self.max_dist, pin_factory=self.factory)
        self.left = DistanceSensor(echo=self.le, trigger=self.lt, max_distance=self.max_dist, pin_factory=self.factory)
        self.right = DistanceSensor(echo=self.fe, trigger=self.rt, max_distance=self.max_dist, pin_factory=self.factory)
        self.back = DistanceSensor(echo=self.be, trigger=self.bt, max_distance=self.max_dist, pin_factory=self.factory)
        
