import logging
from enum import Enum
from utils import units, lazytalonsrx
from components import vision
from magicbot import tunable
from networktables import NetworkTables


class Turret:

    # physical constants
    GEAR_RATIO = 1
    INPUT_PER_OUTPUT = GEAR_RATIO
    OUTPUT_PER_INPUT = 1 / GEAR_RATIO

    SOFT_MIN = -45 * units.radians_per_degree
    SOFT_MAX = 45 * units.radians_per_degree

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
    TURRET_KP = 4
    TURRET_KI = 0.01
    TURRET_KD = 0
    TURRET_KF = 0.08
    TURRET_IZONE = 2 * units.radians_per_degree

    # motion magic config values
    TURRET_MOTION_MAGIC_VELOCITY = tunable(
        90 * units.radians_per_degree * INPUT_PER_OUTPUT
    )
    TURRET_MOTION_MAGIC_ACCEL = tunable(
        90 * units.radians_per_degree * INPUT_PER_OUTPUT
    )

    # tolerance to setpoint error
    VISION_TOLERANCE = 1 * units.radians_per_degree

    # search config
    SEARCH_MIN = (SOFT_MIN + 10) * units.radians_per_degree
    SEARCH_MAX = (SOFT_MAX - 10) * units.radians_per_degree
    SEARCH_SPEED = 0.2

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
        self.output = 0
        self.wanted_heading = 0

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
        self.turret_motor.setIZone(0, self.TURRET_IZONE)
        self.turret_motor.setMotionMagicConfig(
            self.TURRET_MOTION_MAGIC_VELOCITY, self.TURRET_MOTION_MAGIC_ACCEL
        )

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop()

    def isTrackingTarget(self) -> bool:
        return self.mode == self._Mode.TrackingTarget

    def isSearching(self) -> bool:
        return self.mode == self._Mode.Searching

    def isSlewing(self) -> bool:
        return self.isTrackingTarget() or self.isSearching()

    def stop(self) -> None:
        """Stop the turret."""
        self.mode = self._Mode.Idle

    def searchForTarget(self) -> None:
        """Slew the turret back and forth, searhing for a vision target."""
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
        """Get the heading of the turret."""
        return self.turret_motor.getPosition() * self.OUTPUT_PER_INPUT

    def _getHeadingInRange(self) -> float:
        """Get the heading of the turret in the range [-pi, pi]."""
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

    def _checkLimitSwitches(self) -> None:
        """Reset turret heading if limit switch triggered."""
        if self.turret_motor.isFwdLimitSwitchClosed():
            self.turret_motor.zero(self.SOFT_MAX)
        if self.turret_motor.isRevLimitSwitchClosed():
            self.turret_motor.zero(self.SOFT_MIN)

    def updateNetworkTables(self) -> None:
        """Update network table values related to component."""
        self.nt.putValue("heading", self._getHeading() * units.degrees_per_radian)
        self.nt.putValue(
            "heading_in_range", self._getHeadingInRange() * units.degrees_per_radian
        )
        self.nt.putValue("searching", self.mode == self._Mode.Searching)
        self.nt.putValue("tracking_target", self.mode == self._Mode.TrackingTarget)
        self.nt.putValue(
            "heading_error",
            self.turret_motor.getError() * units.degrees_per_radian
            if self.mode == self._Mode.TrackingTarget
            else 0,
        )

    def execute(self):
        self._checkLimitSwitches()
        if self.mode == self._Mode.Idle:
            # stop the turret if idle
            self.turret_motor.set(0)
        elif self.mode == self._Mode.Searching:
            if self.vision.hasTarget():
                # if a vision target is found, track it
                self.mode = self._Mode.TrackingTarget
            else:
                # set initial search direction
                if self.last_mode != self.mode:
                    self.is_searching_reverse = self._getHeadingInRange() <= 0
                # search forwards or reverse
                if self.is_searching_reverse:
                    self.turret_motor.setOutput(-self.SEARCH_SPEED)
                else:
                    self.turret_motor.setOutput(self.SEARCH_SPEED)
                # switch diections if min or max exceeded
                if self._getHeadingInRange() <= self.SEARCH_MIN:
                    self.is_searching_reverse = False
                elif self._getHeadingInRange() >= self.SEARCH_MAX:
                    self.is_searching_reverse = True
        elif self.mode == self._Mode.TrackingTarget:
            if self.vision.hasTarget():
                # if a vision target is found, move to it
                self._setRelativeHeading(-self.vision.getHeading())
            else:
                # if no vision target is found, search for one
                self.mode = self._Mode.Searching
        self.updateNetworkTables()
        self.last_mode = self.mode
