import logging
from wpilib import Timer
from magicbot import tunable
from enum import Enum
from utils import lazypigeonimu, lazytalonsrx, units, pose
from controls import pidf
from components import vision
import numpy as np


class Chassis:

    # chassis physical constants
    TRACK_WIDTH = 24 * units.meters_per_inch
    TRACK_RADIUS = 12 * units.meters_per_inch
    
    WHEEL_RADIUS = 3 * units.meters_per_inch
    WHEEL_CIRCUMFERENCE = 2 * np.pi * WHEEL_RADIUS
    GEAR_RATIO = 10

    # conversions
    RADIANS_PER_METER = (np.pi * 2 * GEAR_RATIO) / WHEEL_CIRCUMFERENCE
    METERS_PER_RADIAN = WHEEL_CIRCUMFERENCE / (np.pi * 2 * GEAR_RATIO)

    # velocity pidf gains
    VELOCITY_LEFT_KP = tunable(0)
    VELOCITY_LEFT_KI = tunable(0)
    VELOCITY_LEFT_KD = tunable(0)
    VELOCITY_LEFT_KF = tunable(0)

    VELOCITY_RIGHT_KP = tunable(0)
    VELOCITY_RIGHT_KI = tunable(0)
    VELOCITY_RIGHT_KD = tunable(0)
    VELOCITY_RIGHT_KF = tunable(0)

    # target tracking pidf gains
    HEADING_KP = 0
    HEADING_KI = 0
    HEADING_KD = 0
    HEADING_KF = 0

    DISTANCE_KP = 0
    DISTANCE_KI = 0
    DISTANCE_KD = 0
    DISTANCE_KF = 0

    # required components
    vision: vision.Vision

    # required devices
    drive_master_left: lazytalonsrx.LazyTalonSRX
    drive_master_right: lazytalonsrx.LazyTalonSRX
    gyro: lazypigeonimu.LazyPigeonIMU

    class _Mode(Enum):
        Idle = 0
        PercentOutput = 1
        Velocity = 2
        Vision = 3

    def __init__(self):
        self.mode = self._Mode.Idle
        self.signal_l = 0
        self.signal_r = 0
        self.last_time = 0

    def setup(self):
        self.heading_pidf = pidf.PIDF(
            self.vision.HEADING_DESIRED,
            self.HEADING_KP,
            self.HEADING_KI,
            self.HEADING_KD,
            self.HEADING_KF,
            True,
            -np.pi,
            np.pi,
        )
        self.distance_pidf = pidf.PIDF(
            self.vision.DISTANCE_DESIRED,
            self.DISTANCE_KP,
            self.DISTANCE_KI,
            self.DISTANCE_KD,
            self.DISTANCE_KF,
        )
        self.drive_master_left.setPIDF(
            0,
            self.VELOCITY_LEFT_KP,
            self.VELOCITY_LEFT_KI,
            self.VELOCITY_LEFT_KD,
            self.VELOCITY_LEFT_KF,
        )
        self.drive_master_right.setPIDF(
            0,
            self.VELOCITY_RIGHT_KP,
            self.VELOCITY_RIGHT_KI,
            self.VELOCITY_RIGHT_KD,
            self.VELOCITY_RIGHT_KF,
        )

    def on_enable(self):
        # TODO remove pidf
        self.heading_pidf = pidf.PIDF(
            self.vision.HEADING_DESIRED,
            self.HEADING_KP,
            self.HEADING_KI,
            self.HEADING_KD,
            self.HEADING_KF,
            True,
            -np.pi,
            np.pi,
        )
        self.distance_pidf = pidf.PIDF(
            self.vision.DISTANCE_DESIRED,
            self.DISTANCE_KP,
            self.DISTANCE_KI,
            self.DISTANCE_KD,
            self.DISTANCE_KF,
        )
        self.drive_master_left.setPIDF(
            0,
            self.VELOCITY_LEFT_KP,
            self.VELOCITY_LEFT_KI,
            self.VELOCITY_LEFT_KD,
            self.VELOCITY_LEFT_KF,
        )
        self.drive_master_right.setPIDF(
            0,
            self.VELOCITY_RIGHT_KP,
            self.VELOCITY_RIGHT_KI,
            self.VELOCITY_RIGHT_KD,
            self.VELOCITY_RIGHT_KF,
        )

    def on_disable(self):
        self.stop()

    def setFromJoystick(self, throttle: float, rotation: float) -> None:
        """Set the output of the motors from joystick values."""
        output_l = throttle - rotation
        output_r = throttle + rotation
        self.setOutput(output_l, output_r)

    def setOutput(self, output_l: float, output_r: float) -> None:
        """Set the output of the motors."""
        self.mode = self._Mode.PercentOutput
        self.signal_l = output_l
        self.signal_r = output_r

    def setVelocity(self, velocity_l: float, velocity_r: float) -> None:
        """Set the velocity of the motors in m/s."""
        self.mode = self._Mode.Velocity
        self.signal_l = velocity_l * self.RADIANS_PER_METER
        self.signal_r = velocity_r * self.RADIANS_PER_METER

    def setRotationalVelocity(self, velocity: float) -> None:
        """Set the rotational velocity of the chassis in rad/s."""
        velocity_l = -velocity * self.TRACK_RADIUS
        velocity_r = velocity * self.TRACK_RADIUS
        self.setVelocity(velocity_l, velocity_r)

    def trackTarget(self) -> None:
        """Track the vision target and drive to it."""
        self.mode = self._Mode.Vision

    def stop(self) -> None:
        """Stop all motor output."""
        self.mode = self._Mode.Idle

    def setBreakMode(self) -> None:
        """Set the motors to break mode."""
        self.drive_master_left.setBreakMode()
        self.drive_master_right.setBreakMode()

    def setCoastMode(self) -> None:
        """Set the motors to coast mode."""
        self.drive_master_left.setCoastMode()
        self.drive_master_right.setCoastMode()

    def execute(self):
        if self.mode == self._Mode.Idle:
            # stop the motors
            self.drive_master_left.setOutput(0.0)
            self.drive_master_right.setOutput(0.0)
        elif self.mode == self._Mode.PercentOutput:
            # set the output of the motors
            self.drive_master_left.setOutput(self.signal_l)
            self.drive_master_right.setOutput(self.signal_r)
        elif self.mode == self._Mode.Velocity:
            # set the velocity of the motors
            self.drive_master_left.setVelocity(self.signal_l)
            self.drive_master_right.setVelocity(self.signal_r)
        elif self.mode == self._Mode.Vision:
            # check if being run for 1st time
            if self.last_time != 0:
                # calculate pidf outputs
                cur_time = Timer.getFPGATimestamp()
                dt = cur_time - self.last_time

                distance_error = self.vision.getDistanceError()
                heading_error = self.vision.getHeadingError()

                distance_adjust = self.distance_pidf.update(distance_error, dt)
                heading_adjust = self.heading_pidf.update(heading_error, dt)

                # calculate wheel velocities and set motor outputs
                self.signal_l = distance_adjust - heading_adjust
                self.signal_r = distance_adjust + heading_adjust

                self.drive_master_left.setVelocity(self.signal_l)
                self.drive_master_right.setVelocity(self.signal_r)

                self.last_time = Timer.getFPGATimestamp()
