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

    # motion magic pidf gains
    TURRET_KP = tunable(24)
    TURRET_KI = tunable(0)
    TURRET_KD = tunable(100)
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

    # search config
    SEARCH_MIN = -80 * units.degrees_per_radian
    SEARCH_MAX = 80 * units.degrees_per_radian
    SEARCH_SPEED = 0.5

    # required devices
    turret_motor: lazytalonsrx.LazyTalonSRX

    # required components
    vision: vision.Vision

    class _Mode(Enum):
        Idle = 0
        Searching = 1
        TrackingTarget = 2

    def __init__(self):
        self.mode = self._Mode.Idle
        self.last_mode = self.mode
        self.output = 0
        self.nt = NetworkTables.getTable(
            f"/components/{self.__class__.__name__.lower()}"
        )
        self.is_searching_reverse = False

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
        
    def isSlewing(self) -> bool:
        return self.mode == self._Mode.Searching or self.mode == self._Mode.TrackingTarget

    def stop(self) -> None:
        """Stop the turret."""
        self.mode = self._Mode.Idle

    def searchForTarget(self) -> None:
        self.mode = self._Mode.Searching

    def trackTarget(self) -> None:
        """Move turret to actively track the vision target."""
        self.mode = self._Mode.TrackingTarget


    def _isWithinSoftLimits(self, angle: float) -> bool:
        """Is the given angle within the soft limits."""
        return self.SOFT_MIN <= angle and angle <= self.SOFT_MAX

    def isWithinSoftLimits(self) -> bool:
        """Is the turret heading within the soft limits."""
        heading = self._getHeadingInRange()
        return self._isWithinSoftLimits(heading)

    def isAtVisionTarget(self) -> bool:
        """Is the turret centered on the vision target."""
        return (
            self.mode == self._Mode.TrackingTarget
            and self.vision.isHeadingInFineRange()
        )

    def isReady(self) -> bool:
        """Is the turret ready to shoot balls."""
        return self.isAtVisionTarget()

    def _getHeading(self) -> float:
        return self.turret_motor.getPosition() * self.OUTPUT_PER_INPUT

    def _getHeadingInRange(self):
        return units.angle_range(self._getHeading())

    def _setRelativeHeading(self, heading: float) -> float:
        """Set the heading of the turret relative to the current heading."""
        heading += self._getHeading()
        self._setAbsoluteHeading(heading)

    def _setAbsoluteHeading(self, heading: float) -> float:
        """Set the absolute heading of the turret."""
        heading *= self.INPUT_PER_OUTPUT
        if self._isWithinSoftLimits(heading):
            self.turret_motor.setMotionMagicPosition(heading)
        else:
            logging.warning(
                f"Trying to set turret at {heading} which is outside of soft limits"
            )

    def _checkLimitSwitches(self):
        if self.turret_motor.isFwdLimitSwitchClosed():
            self.turret_motor.zero(self.SOFT_MAX)
        if self.turret_motor.isRevLimitSwitchClosed():
            self.turret_motor.zero(self.SOFT_MIN)

    def updateNetworkTables(self):
        self.nt.putValue("heading", self._getHeading() * units.degrees_per_radian)
        self.nt.putValue(
            "heading_in_range", self._getHeadingInRange() * units.degrees_per_radian
        )
        self.nt.putValue("searching", self.mode == self._Mode.Searching)
        self.nt.putValue("tracking_target", self.mode == self._Mode.TrackingTarget)

    def execute(self):
        self._checkLimitSwitches()
        if self.mode == self._Mode.Idle:
            self.turret_motor.set(0)
        elif self.mode == self._Mode.Searching:
            if self.vision.hasTarget():
                self.mode = self._Mode.TrackingTarget
            else:
                if self.last_mode != self.mode:
                    self.is_searching_reverse = self._getHeadingInRange() <= 0
                if self.is_searching_reverse:
                    self.turret_motor.setOutput(-self.SEARCH_SPEED)
                else:
                    self.turret_motor.setOutput(self.SEARCH_SPEED)
                if self._getHeadingInRange() <= self.SEARCH_MIN:
                    self.is_searching_reverse == False
                elif self._getHeadingInRange >= self.SEARCH_MAX:
                    self.is_searching_reverse == True
        elif self.mode == self._Mode.TrackingTarget:
            if self.vision.hasTarget():
                if self.isReady():
                    self.turret_motor.set(0)
                else:
                    self._setRelativeHeading(-self.vision.getHeading())
            else:
                self.mode = self._Mode.Searching
        self.updateNetworkTables()
        self.last_mode = self.mode
