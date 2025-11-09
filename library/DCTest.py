#!/usr/bin/python
from Emakefun_MotorHAT import Emakefun_MotorHAT, Emakefun_DCMotor, Emakefun_Servo

import time
import atexit

vel = 25
# create a default object, no changes to I2C address or frequency
mh = Emakefun_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
	mh.getMotor(1).run(Emakefun_MotorHAT.RELEASE)
	mh.getMotor(2).run(Emakefun_MotorHAT.RELEASE)
	mh.getMotor(3).run(Emakefun_MotorHAT.RELEASE)
	mh.getMotor(4).run(Emakefun_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

################################# DC motor test!
myMotorL = mh.getMotor(1)
myMotorR = mh.getMotor(2)

# set the speed to start, from 0 (off) to 255 (max speed)
myMotorL.setSpeed(vel)
myMotorR.setSpeed(vel)
myMotorL.run(Emakefun_MotorHAT.BACKWARD)
myMotorR.run(Emakefun_MotorHAT.FORWARD)
time.sleep(3)
# turn on motor
myMotorL.run(Emakefun_MotorHAT.RELEASE)
myMotorR.run(Emakefun_MotorHAT.RELEASE)

