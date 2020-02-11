#!/usr/bin/env python

import wpilib
from magicbot import MagicRobot
from utils import lazytalonsrx, lazypigeonimu, pose
from components import chassis, intake, tower, turret, shooter, vision
import rev


class Robot(MagicRobot):
    DRIVE_SLAVE_LEFT_ID = 1
    DRIVE_MASTER_LEFT_ID = 2

    DRIVE_SLAVE_RIGHT_ID = 3
    DRIVE_MASTER_RIGHT_ID = 4

    INTAKE_ID = 5

    LOW_TOWER_ID = 6
    HIGH_TOWER_ID = 7

    TURRET_ID = 8
    SHOOTER_LEFT_ID = 9
    SHOOTER_RIGHT_ID = 10

    CLIMB_WINCH_LEFT_ID = 11
    CLIMB_WINCH_RIGHT_ID = 12

    BUDDY_WINCH_LEFT_ID = 13
    BUDDY_WINCH_RIGHT_ID = 14

    TROLLEY_ARM_ID = 15
    TROLLEY_ID = 16

    chassis: chassis.Chassis
    intake: intake.Intake
    tower: tower.Tower
    turret: turret.Turret
    shooter: shooter.Shooter
    vision: vision.Vision

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""
        self.drive_slave_left = lazytalonsrx.LazyTalonSRX(self.DRIVE_SLAVE_LEFT_ID)
        self.drive_master_left = lazytalonsrx.LazyTalonSRX(self.DRIVE_MASTER_LEFT_ID)
        self.drive_master_left.follow(self.drive_slave_left)

        self.drive_slave_right = lazytalonsrx.LazyTalonSRX(self.DRIVE_SLAVE_RIGHT_ID)
        self.drive_master_right = lazytalonsrx.LazyTalonSRX(self.DRIVE_MASTER_RIGHT_ID)
        self.drive_master_right.follow(self.drive_slave_right)
        self.drive_master_right.setInverted(True)

        self.drive_master_left.setEncoderConfig(lazytalonsrx.FalconEncoder, False)
        self.drive_master_right.setEncoderConfig(lazytalonsrx.FalconEncoder, False)

        self.gyro = lazypigeonimu.LazyPigeonIMU(self.drive_master_right)

        self.intake_motor = lazytalonsrx.LazyTalonSRX(self.INTAKE_ID)

        self.low_tower_motor = lazytalonsrx.LazyTalonSRX(self.LOW_TOWER_ID)
        self.high_tower_motor = lazytalonsrx.LazyTalonSRX(self.HIGH_TOWER_ID)

        self.turret_motor = lazytalonsrx.LazyTalonSRX(self.TURRET_ID)
        self.turret_motor.setEncoderConfig(lazytalonsrx.TurretEncoder, False)

        self.shooter_motor_left = rev.CANSparkMax(
            self.SHOOTER_LEFT_ID, rev.MotorType.kBrushless
        )
        self.shooter_motor_right = rev.CANSparkMax(
            self.SHOOTER_RIGHT_ID, rev.MotorType.kBrushless
        )
        self.shooter_motor_right.follow(self.shooter_motor_left, True)

        self.trolley_arm = lazytalonsrx.LazyTalonSRX(self.TROLLEY_ARM_ID)
        self.trolley_motor = lazytalonsrx.LazyTalonSRX(self.TROLLEY_ID)

        self.driver = wpilib.Joystick(0)
        self.operator = wpilib.Joystick(1)

        self.global_pose = pose.Pose(0, 0, 0)

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
           actions"""
        try:
            # driver joystick control of chassis
            if self.chassis.mode != self.chassis._Mode.Vision:
                throttle = -self.driver.getY()
                rotation = -self.driver.getZ()
                self.chassis.setFromJoystick(throttle, rotation)

            # driver control of chassis target tracking
            if self.driver.getRawButtonPressed(0):
                if self.chassis.mode == self.chassis._Mode.Vision:
                    self.chassis.stop()
                else:
                    self.chassis.trackTarget()

            # operator control of turret target tracking
            if self.operator.getRawButtonPressed(0):
                if self.turret.is_tracking_target:
                    self.turret.stop()
                else:
                    self.turret.trackTarget()

            # operator control of intake
            if self.operator.getRawButtonPressed(5):
                if self.intake.is_intaking:
                    self.intake.stop()
                else:
                    self.intake.intake()

            # operator control of tower indexing
            if self.operator.getRawButtonPressed(6):
                if self.tower.is_indexing:
                    self.tower.stop()
                elif not self.tower.isFullyLoaded():
                    self.tower.index()

            # operator control of shooter spinning
            if self.operator.getRawButton(7):
                if self.shooter.is_shooting:
                    self.shooter.stop()
                else:
                    self.shooter.shoot()

        except:
            self.onException()


if __name__ == "__main__":
    wpilib.run(Robot)
