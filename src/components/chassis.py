import logging
from enum import Enum
from utils import lazypigeonimu, lazytalonsrx, units, pose
from controls import pidf
from components import vision
import numpy as np


class Chassis:

    TRACK_WIDTH = 24 * units.meters_per_inch
    TRACK_RADIUS = 12 * units.meters_per_inch

    WHEEL_RADIUS = 3 * units.meters_per_inch
    WHEEL_CIRCUMFERENCE = 2 * np.pi * WHEEL_RADIUS
    GEAR_RATIO = 10

    RADIANS_PER_METER = (np.pi * 2 * GEAR_RATIO) / WHEEL_CIRCUMFERENCE
    METERS_PER_RADIAN = WHEEL_CIRCUMFERENCE / (np.pi * 2 * GEAR_RATIO)

    HEADING_KP = 0
    HEADING_KI = 0
    HEADING_KD = 0
    HEADING_KF = 0

    DISTANCE_KP = 0
    DISTANCE_KI = 0
    DISTANCE_KD = 0
    DISTANCE_KF = 0

    vision: vision.Vision

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

    def on_enable(self):
        pass

    def setFromJoystick(self, throttle: float, rotation: float) -> None:
        """Set the output of the motors from joystick values."""
        output_l = throttle - rotation
        output_r = throttle + rotation
        self.setOutput(output_l, output_r)

    def setOutput(self, output_l: float, output_r: float) -> None:
        """Set the output of the motors from percent values."""
        self.mode = self._Mode.PercentOutput
        self.signal_l = output_l
        self.signal_r = output_r

    def setVelocity(self, velocity_l: float, velocity_r: float) -> None:
        """Set the output of the motors from percent values."""
        self.mode = self._Mode.Velocity
        self.signal_l = velocity_l * self.RADIANS_PER_METER
        self.signal_r = velocity_r * self.RADIANS_PER_METER

    def setRotationalVelocity(self, velocity: float) -> None:
        """Set the output of the motors from percent values."""
        velocity_l = -velocity * self.TRACK_RADIUS
        velocity_r = velocity * self.TRACK_RADIUS
        self.setVelocity(velocity_l, velocity_r)

    def trackTarget(self):
        self.mode = self._Mode.Vision

    def stop(self):
        self.mode = self._Mode.Idle

    def setBreakMode(self):
        self.drive_master_left.setBreakMode()
        self.drive_master_right.setBreakMode()

    def setCoastMode(self):
        self.drive_master_left.setCoastMode()
        self.drive_master_right.setCoastMode()

    def execute(self):
        if self.mode == self._Mode.Idle:
            self.drive_master_left.setOutput(0.0)
            self.drive_master_right.setOutput(0.0)
        elif self.mode == self._Mode.PercentOutput:
            self.drive_master_left.setOutput(self.signal_l)
            self.drive_master_right.setOutput(self.signal_r)
        elif self.mode == self._Mode.Velocity:
            self.drive_master_left.setVelocity(self.signal_l)
            self.drive_master_right.setVelocity(self.signal_r)
        elif self.mode == self._Mode.Vision:
            if self.last_time != 0:
                self.cur_time = Timer.getFPGATimestamp()
                dt = self.cur_time - self.last_time

                distance_error = self.vision.getDistanceError()
                heading_error = self.vision.getHeadingError()

                heading_adjust = self.heading_pidf.update(heading_error, dt)
                distance_adjust = self.distance_pidf.update(distance_error, dt)

                self.signal_l = distance_adjust - heading_adjust
                self.signal_r = distance_adjust + heading_adjust

                self.drive_master_left.setVelocity(self.signal_l)
                self.drive_master_right.setVelocity(self.signal_r)

                self.last_time = Timer.getFPGATimestamp()
