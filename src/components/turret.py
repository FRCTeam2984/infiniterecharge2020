import logging
from enum import Enum
import rev
from utils import units, lazytalonsrx
import numpy as np
from controls import pidf
import wpilib
from components import vision


class Turret:

    GEAR_RATIO = 10
    INPUT_RADS_PER_OUTPUT_RAD = GEAR_RATIO
    OUTPUT_RADS_PER_INPUT_RAD = 1 / GEAR_RATIO

    SOFT_MIN = -45 * units.radians_per_degree
    SOFT_MAX = 45 * units.radians_per_degree

    TURRET_POS_KP = 0.1
    TURRET_POS_KI = 0
    TURRET_POS_KD = 0
    TURRET_POS_KF = 0
    TURRET_MOTION_MAGIC_VELOCITY = (
        90 * units.radians_per_degree * INPUT_RADS_PER_OUTPUT_RAD
    )
    TURRET_MOTION_MAGIC_ACCEL = (
        45 * units.radians_per_degree * INPUT_RADS_PER_OUTPUT_RAD
    )

    VISION_TOLERANCE = 1 * units.radians_per_degree

    turret_motor: lazytalonsrx.LazyTalonSRX
    vision: vision.Vision

    def __init__(self):
        self.tracking_target = False

    def setup(self):
        self.turret_motor.setMotionMagicConfig(
            self.TURRET_MOTION_MAGIC_VELOCITY, self.TURRET_MOTION_MAGIC_ACCEL
        )

    def on_enable(self):
        pass

    def trackTarget(self) -> None:
        """Start spinning the shooter."""
        self.tracking_target = True

    def stop(self) -> None:
        """Stop the shooter."""
        self.tracking_target = False

    def getVisionTargetHeading(self):
        cur_heading = self.turret_motor.getPosition() * self.OUTPUT_RADS_PER_INPUT_RAD
        new_heading = (
            cur_heading + self.vision.getHeading()
        ) * self.INPUT_RADS_PER_OUTPUT_RAD
        return new_heading

    def _isWithinSoftLimits(self, angle: float):
        return self.SOFT_MIN <= angle and angle <= self.SOFT_MAX

    def isWithinSoftLimits(self):
        heading = self.turret_motor.getPosition() * self.OUTPUT_RADS_PER_INPUT_RAD
        return self._isWithinSoftLimits(heading)

    def isAtVisionTarget(self):
        vision_heading = self.getVisionTargetHeading()
        return (
            self.tracking_target
            and abs(vision_heading - self.turret_motor.getPosition())
            <= self.VISION_TOLERANCE
        )

    def isReady(self):
        return self.isAtVisionTarget()

    def execute(self):
        if self.tracking_target:
            new_heading = self.getVisionTargetHeading()
            if self._isWithinSoftLimits(new_heading):
                self.turret_motor.setPosition(new_heading)
            else:
                logging.warning(
                    f"Trying to set turret at {new_heading} which is outside of soft limits"
                )
        else:
            self.turret_motor.setOutput(0)

