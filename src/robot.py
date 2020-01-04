#!/usr/bin/env python3

import wpilib
import ctre
from magicbot import MagicRobot
from utils import lazytalonsrx, lazypigeonimu
from components.chassis import Chassis
import numpy as np


class Robot(MagicRobot):
    DS_L_ID = 2
    DM_L_ID = 3
    DS_R_ID = 0
    DM_R_ID = 1

    chassis: Chassis

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""
        self.ds_l = lazytalonsrx.LazyTalonSRX(self.DS_L_ID)
        self.dm_l = lazytalonsrx.LazyTalonSRX(self.DM_L_ID)
        self.dm_l.follow(self.ds_l)

        self.ds_r = lazytalonsrx.LazyTalonSRX(self.DS_R_ID)
        self.dm_r = lazytalonsrx.LazyTalonSRX(self.DM_R_ID)
        self.dm_r.follow(self.ds_r)

        self.ds_r.setInverted(True)
        self.dm_r.setInverted(True)

        self.gyro = lazypigeonimu.LazyPigeonIMU(self.dm_r)

        self.driver = wpilib.joystick.Joystick(0)

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
           actions"""
        try:
            throttle = -self.driver.getY()
            rotation = -self.driver.getZ()

            self.chassis.setFromJoystick(throttle, rotation)
        except:
            self.onException()


if __name__ == "__main__":
    wpilib.run(Robot)
