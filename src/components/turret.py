import logging
from enum import Enum
import rev
from utils import units, lazytalonsrx
import numpy as np
from controls import pidf
import wpilib
from components import vision
from magicbot import tunable, feedback


class Turret:

    # turret physical constants
    GEAR_RATIO = 462
    INPUT_RADS_PER_OUTPUT_RAD = GEAR_RATIO
    OUTPUT_RADS_PER_INPUT_RAD = 1 / GEAR_RATIO

    SOFT_MIN = -45 * units.radians_per_degree
    SOFT_MAX = 45 * units.radians_per_degree

    # motion magic pidf gains
    TURRET_KP = tunable(0)
    TURRET_KI = tunable(0)
    TURRET_KD = tunable(0)
    TURRET_KF = tunable(0)

    # motion magic config values
    TURRET_MOTION_MAGIC_VELOCITY = (
        90 * units.radians_per_degree * INPUT_RADS_PER_OUTPUT_RAD
    )
    TURRET_MOTION_MAGIC_ACCEL = (
        45 * units.radians_per_degree * INPUT_RADS_PER_OUTPUT_RAD
    )

    # tolerance to setpoint error
    VISION_TOLERANCE = 1 * units.radians_per_degree

    # required devices
    turret_motor: lazytalonsrx.LazyTalonSRX

    # required components
    vision: vision.Vision

    def __init__(self):
        self.is_tracking_target = False

    def setup(self):
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
        """Get the heading the turret should be at to be centered on the vision target."""
        cur_heading = self.turret_motor.getPosition() * self.OUTPUT_RADS_PER_INPUT_RAD
        new_heading = cur_heading + self.vision.getHeading()
        return new_heading

    def _isWithinSoftLimits(self, angle: float) -> bool:
        """Is the given angle within the soft limits."""
        return self.SOFT_MIN <= angle and angle <= self.SOFT_MAX

    def isWithinSoftLimits(self) -> bool:
        """Is the turret heading within the soft limits."""
        heading = self.turret_motor.getPosition() * self.OUTPUT_RADS_PER_INPUT_RAD
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
        heading *= self.INPUT_RADS_PER_OUTPUT_RAD
        if self._isWithinSoftLimits(heading):
            self.turret_motor.setPosition(heading)
        else:
            logging.warning(
                f"Trying to set turret at {heading} which is outside of soft limits"
            )

    @feedback
    def get_heading(self):
        return self.turret_motor.getPosition() * self.OUTPUT_RADS_PER_INPUT_RAD

    @feedback
    def get_vision_target_heading(self):
        return self.getVisionTargetHeading()

    @feedback
    def get_is_tracking_target(self):
        return self.is_tracking_target

    def execute(self):
        if self.is_tracking_target:
            # vision_heading = self.getVisionTargetHeading()
            # self._setHeading(vision_heading)
            self.turret_motor.setOutput(0.5)
        else:
            self.turret_motor.setOutput(0)
