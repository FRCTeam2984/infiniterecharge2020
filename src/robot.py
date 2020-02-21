#!/usr/bin/env python

import rev
import wpilib
from components import chassis, flywheel, intake, leds, tower, turret, vision
from magicbot import MagicRobot
from statemachines import alignchassis, shooter
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
    FLYWHEEL_LEFT_ID = 9
    FLYWHEEL_RIGHT_ID = 10

    CLIMB_WINCH_LEFT_ID = 11
    CLIMB_WINCH_RIGHT_ID = 12

    BUDDY_WINCH_LEFT_ID = 13
    BUDDY_WINCH_RIGHT_ID = 14

    CLIMB_ARM_ID = 15
    TROLLEY_ID = 16

    chassis: chassis.Chassis
    intake: intake.Intake
    tower: tower.Tower
    turret: turret.Turret
    flywheel: flywheel.Flywheel
    vision: vision.Vision
    shooter: shooter.Shooter
    alignchassis: alignchassis.AlignChassis
    leds: leds.LED

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

        self.flywheel_motor_left = rev.CANSparkMax(
            self.FLYWHEEL_LEFT_ID, rev.MotorType.kBrushless
        )
        # self.flywheel_motor_right = rev.CANSparkMax(
        #     self.FLYWHEEL_RIGHT_ID, rev.MotorType.kBrushless
        # )
        # self.flywheel_motor_right.follow(self.flywheel_motor_left, True)

        self.climb_winch_left = lazytalonsrx.LazyTalonSRX(self.CLIMB_WINCH_LEFT_ID)
        self.climb_winch_right = lazytalonsrx.LazyTalonSRX(self.CLIMB_WINCH_RIGHT_ID)
        self.climb_winch_right.follow(self.climb_winch_left)

        self.buddy_winch_left = lazytalonsrx.LazyTalonSRX(self.BUDDY_WINCH_LEFT_ID)
        self.buddy_winch_right = lazytalonsrx.LazyTalonSRX(self.BUDDY_WINCH_RIGHT_ID)
        self.buddy_winch_right.follow(self.buddy_winch_left)

        self.climb_arm = lazytalonsrx.LazyTalonSRX(self.CLIMB_ARM_ID)
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
            # # driver joystick control of chassis
            # driver control of chassis target tracking
            if self.driver.getRawButton(1):
                self.alignchassis.align()
            else:
                throttle = self.driver.getY()
                rotation = self.driver.getZ()
                self.chassis.setFromJoystick(throttle, rotation)
            # # operator control of shooter
            # if self.operator.getRawButtonPressed(1):
            #     if self.shooter.is_shooting:
            #         self.shooter.stop()
            #     else:
            #         self.shooter.shoot()

            # # operator control of indexer
            # if self.operator.getRawButtonPressed(5):
            #     if self.indexer.is_indexing:
            #         self.indexer.stop()
            #     else:
            #         self.indexer.index()

        except:
            self.onException()


if __name__ == "__main__":
    wpilib.run(Robot)
