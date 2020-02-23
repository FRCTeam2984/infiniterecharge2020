#!/usr/bin/env python

import rev
import wpilib
from magicbot import MagicRobot

from components import chassis, flywheel, intake, leds, tower, turret, vision, winch, buddywinch, trolley, slider
from statemachines import alignchassis, shooter, climb
from utils import lazypigeonimu, lazytalonfx, lazytalonsrx


class Robot(MagicRobot):
    DRIVE_SLAVE_LEFT_ID = 1
    DRIVE_MASTER_LEFT_ID = 2
    DRIVE_SLAVE_RIGHT_ID = 3
    DRIVE_MASTER_RIGHT_ID = 4

    INTAKE_ID = 5

    LOW_TOWER_ID = 6
    HIGH_TOWER_ID = 7

    TURRET_ID = 8
    FLYWHEEL_SLAVE_ID = 9
    FLYWHEEL_MASTER_ID = 10

    CLIMB_WINCH_SLAVE_ID = 11
    CLIMB_WINCH_MASTER_ID = 12

    BUDDY_WINCH_SLAVE_ID = 13
    BUDDY_WINCH_MASTER_ID = 14

    SLIDER_ID = 15
    TROLLEY_ID = 16

    chassis: chassis.Chassis
    intake: intake.Intake
    tower: tower.Tower
    turret: turret.Turret
    flywheel: flywheel.Flywheel
    vision: vision.Vision
    slider: slider.Slider
    winch: winch.Winch
    buddywinch: buddywinch.BuddyWinch
    trolley: trolley.Trolley
    leds: leds.LED

    shooter: shooter.Shooter
    alignchassis: alignchassis.AlignChassis
    climb: climb.Climb
    

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""
        # setup master and slave drive motors
        self.drive_slave_left = lazytalonfx.LazyTalonFX(self.DRIVE_SLAVE_LEFT_ID)
        self.drive_master_left = lazytalonfx.LazyTalonFX(self.DRIVE_MASTER_LEFT_ID)
        self.drive_master_left.follow(self.drive_slave_left)

        self.drive_slave_right = lazytalonfx.LazyTalonFX(self.DRIVE_SLAVE_RIGHT_ID)
        self.drive_master_right = lazytalonfx.LazyTalonFX(self.DRIVE_MASTER_RIGHT_ID)
        self.drive_master_right.follow(self.drive_slave_right)

        # setup actuator motors
        self.intake_motor = lazytalonsrx.LazyTalonSRX(self.INTAKE_ID)

        self.low_tower_motor = lazytalonsrx.LazyTalonSRX(self.LOW_TOWER_ID)
        self.high_tower_motor = lazytalonsrx.LazyTalonSRX(self.HIGH_TOWER_ID)

        self.turret_motor = lazytalonsrx.LazyTalonSRX(self.TURRET_ID)
        self.turret_motor.setEncoderConfig(lazytalonsrx.CTREMag, True)

        self.flywheel_master = rev.CANSparkMax(
            self.FLYWHEEL_MASTER_ID, rev.MotorType.kBrushless
        )
        # self.flywheel_slave = rev.CANSparkMax(
        #     self.FLYWHEEL_SLAVE_ID, rev.MotorType.kBrushless
        # )
        # self.flywheel_slave.follow(self.flywheel_master, True)

        self.climb_winch_slave = lazytalonsrx.LazyTalonSRX(self.CLIMB_WINCH_SLAVE_ID)
        self.climb_winch_master = lazytalonsrx.LazyTalonSRX(self.CLIMB_WINCH_MASTER_ID)
        self.climb_winch_slave.follow(self.climb_winch_master)

        self.buddy_winch_slave = lazytalonsrx.LazyTalonSRX(self.BUDDY_WINCH_SLAVE_ID)
        self.buddy_winch_master = lazytalonsrx.LazyTalonSRX(self.BUDDY_WINCH_MASTER_ID)
        self.buddy_winch_slave.follow(self.buddy_winch_master)

        self.slider_motor = lazytalonsrx.LazyTalonSRX(self.SLIDER_ID)
        self.trolley_motor = lazytalonsrx.LazyTalonSRX(self.TROLLEY_ID)

        # setup imu
        self.imu = lazypigeonimu.LazyPigeonIMU(self.intake_motor)

        # setup leds
        self.led = wpilib.AddressableLED(0)

        # setup joysticks
        self.driver = wpilib.Joystick(0)
        self.operator = wpilib.Joystick(1)

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
           actions"""
        try:
            # TODO remove temp controls
            if self.operator.getRawButtonPressed(1):
                if self.flywheel.is_spinning:
                    self.flywheel.stop()
                else:
                    self.flywheel.setRPM(0)
            if self.operator.getRawButton(2):
                self.shooter.engage()

            # if self.operator.getRawButtonPressed(2):
            #     if self.flywheel.is_spinning:
            #         self.flywheel.stop()
            #     else:
            #         # self.flywheel.setRPM(zdesired_rpm)

            #################
            # real controls #
            #################

            ##########
            # driver #
            ##########
            # # chassis target tracking
            # if self.driver.getRawButton(1):
            #     self.alignchassis.align()
            # else:
            #     throttle = self.driver.getY()
            #     rotation = self.driver.getZ()
            #     self.chassis.setFromJoystick(throttle, rotation)

            ############
            # operator #
            ############
            # # shooter
            # if self.operator.getRawButton(1):
            #     self.shooter.shoot()

            # # indexer
            # if self.operator.getRawButton(2):
            #     self.indexer.index()  
            # else:
            #     self.indexer.stop()

            # # trolley
            # self.trolley.setFromJoystick(self.operator.getY())

            # climb
            if self.operator.getRawButton(5):
                self.climb.reachHook()

            # climb
            if self.operator.getRawButton(7):
                self.climb.activateWinch()

            # # buddy climb
            # if self.operator.getRawButton(7):
            #     self.buddywinch.winch()
            # else:
            #     self.buddywinch.stop()

        except:
            self.onException()


if __name__ == "__main__":
    wpilib.run(Robot)
