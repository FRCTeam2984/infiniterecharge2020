import logging
from enum import Enum
import rev
from utils import units, lazytalonsrx
import numpy as np
from controls import pidf
import wpilib
from components import vision
from magicbot import tunable, feedback
from networktables import NetworkTables


class Turret:

    # physical constants
    GEAR_RATIO = 1
    INPUT_PER_OUTPUT = GEAR_RATIO
    OUTPUT_PER_INPUT = 1 / GEAR_RATIO

    SOFT_MIN = -85 * units.radians_per_degree
    SOFT_MAX = 85 * units.radians_per_degree

    # motor config
    TIMEOUT = lazytalonsrx.LazyTalonSRX.TIMEOUT
    INVERTED = False
    STATUS_FRAME = 10

    CLOSED_LOOP_RAMP = 0.5
    OPEN_LOOP_RAMP = 0.5
    PEAK_CURRENT = 50
    PEAK_CURRENT_DURATION = 1000
    CONTINUOUS_CURRENT = 25

    # motor coefs
    TURRET_KS = 0  # V
    TURRET_KV = 0  # V / (rad / s)
    TURRET_KA = 0  # V / (rad / s^2)

    # motion magic pidf gains
    TURRET_KP = tunable(0)
    TURRET_KI = tunable(0)
    TURRET_KD = tunable(0)
    TURRET_KF = tunable(0)

    # motion magic config values
    TURRET_MOTION_MAGIC_VELOCITY = tunable(
        30 * units.radians_per_degree * INPUT_PER_OUTPUT
    )
    TURRET_MOTION_MAGIC_ACCEL = tunable(
        30 * units.radians_per_degree * INPUT_PER_OUTPUT
    )

    # tolerance to setpoint error
    VISION_TOLERANCE = 1 * units.radians_per_degree

    # required devices
    turret_motor: lazytalonsrx.LazyTalonSRX

    # required components
    vision: vision.Vision

    def __init__(self):
        self.is_tracking_target = False
        self.output = 0
        self.nt = NetworkTables.getTable(
            f"/components/{self.__class__.__name__.lower()}"
        )

    def setup(self):
        # self.turret_motor.setStatusFramePeriod(
        #     lazytalonsrx.LazyTalonSRX.StatusFrame.Status_2_Feedback0,
        #     self.STATUS_FRAME,
        #     self.TIMEOUT,
        # )

        self.turret_motor.configOpenloopRamp(self.OPEN_LOOP_RAMP, self.TIMEOUT)
        self.turret_motor.configClosedloopRamp(self.CLOSED_LOOP_RAMP, self.TIMEOUT)

        self.turret_motor.enableCurrentLimit(True)
        self.turret_motor.configPeakCurrentLimit(self.PEAK_CURRENT, self.TIMEOUT)
        self.turret_motor.configPeakCurrentDuration(
            self.PEAK_CURRENT_DURATION, self.TIMEOUT
        )
        self.turret_motor.configContinuousCurrentLimit(
            self.CONTINUOUS_CURRENT, self.TIMEOUT
        )

        self.turret_motor.setPIDF(
            0, self.TURRET_KP, self.TURRET_KI, self.TURRET_KD, self.TURRET_KF
        )
        self.turret_motor.setMotionMagicConfig(
            self.TURRET_MOTION_MAGIC_VELOCITY, self.TURRET_MOTION_MAGIC_ACCEL
        )

    def on_enable(self):
        # TODO remove this
        self.turret_motor.setPIDF(
            0, self.TURRET_KP, self.TURRET_KI, self.TURRET_KD, self.TURRET_KF
        )
        self.turret_motor.setMotionMagicConfig(
            self.TURRET_MOTION_MAGIC_VELOCITY, self.TURRET_MOTION_MAGIC_ACCEL
        )

    def on_disable(self):
        self.stop()

    def trackTarget(self) -> None:
        """Move turret to actively track the vision target."""
        self.is_tracking_target = True

    def stop(self) -> None:
        """Stop the turret."""
        self.is_tracking_target = False

    def getVisionTargetHeading(self) -> float:
        """Get the heading of the target relative to the turret"""
        cur_heading = self.turret_motor.getPosition() * self.OUTPUT_PER_INPUT
        new_heading = cur_heading + self.vision.getHeading()
        return new_heading

    def _isWithinSoftLimits(self, angle: float) -> bool:
        """Is the given angle within the soft limits."""
        return self.SOFT_MIN <= angle and angle <= self.SOFT_MAX

    def isWithinSoftLimits(self) -> bool:
        """Is the turret heading within the soft limits."""
        heading = self.getHeadingInRange()
        return self._isWithinSoftLimits(heading)

    def isAtVisionTarget(self) -> bool:
        """Is the turret centered on the vision target."""
        return (
            self.is_tracking_target
            and abs(self.getVisionTargetHeading() - self.turret_motor.getPosition())
            <= self.VISION_TOLERANCE
        )

    def isReady(self) -> bool:
        """Is the turret ready to shoot balls."""
        return self.isAtVisionTarget()

    def _setHeading(self, heading: float) -> float:
        """Set the heading of the turret."""
        heading *= self.INPUT_PER_OUTPUT
        if self._isWithinSoftLimits(heading):
            self.turret_motor.setMotionMagicPosition(heading)
        else:
            logging.warning(
                f"Trying to set turret at {heading} which is outside of soft limits"
            )

    def getHeadingInRange(self):
        return units.angle_range(self.turret_motor.getPosition())

    def updateNetworkTables(self):
        self.nt.putValue(
            "heading_in_range", self.getHeadingInRange() * units.degrees_per_radian
        )
        self.nt.putValue(
            "heading",
            self.turret_motor.getPosition()
            * self.OUTPUT_PER_INPUT
            * units.degrees_per_radian,
        )
        self.nt.putValue("tracking_target", self.is_tracking_target)

    def execute(self):
        if self.is_tracking_target:
            vision_heading = self.getVisionTargetHeading()
            self._setHeading(vision_heading)
        else:
            self.turret_motor.setOutput(0)
        self.updateNetworkTables()
