import logging
from enum import Enum

from magicbot import tunable
from networktables import NetworkTables

from components import vision
from utils import lazypigeonimu, lazytalonsrx, units


class Turret:

    # physical constants
    GEAR_RATIO = 1
    INPUT_PER_OUTPUT = GEAR_RATIO
    OUTPUT_PER_INPUT = 1 / GEAR_RATIO

    SOFT_MIN = -70 * units.radians_per_degree
    SOFT_MAX = 70 * units.radians_per_degree

    # motor config
    TIMEOUT = lazytalonsrx.LazyTalonSRX.TIMEOUT
    INVERTED = False
    STATUS_FRAME = 10

    CLOSED_LOOP_RAMP = 0
    OPEN_LOOP_RAMP = 0
    PEAK_CURRENT = 50
    PEAK_CURRENT_DURATION = 1000
    CONTINUOUS_CURRENT = 25

    # position pidf gains
    TURRET_KP = 4
    TURRET_KI = 0.01
    TURRET_KD = 0
    TURRET_KF = 0.08
    TURRET_IZONE = 2 * units.radians_per_degree
    HEADING_TOLERANCE = 0.5 * units.radians_per_degree

    # motion magic config values
    TURRET_MOTION_MAGIC_VELOCITY = tunable(
        90 * units.radians_per_degree * INPUT_PER_OUTPUT
    )
    TURRET_MOTION_MAGIC_ACCEL = tunable(
        90 * units.radians_per_degree * INPUT_PER_OUTPUT
    )

    # tolerance to setpoint error
    VISION_TOLERANCE = 1 * units.radians_per_degree

    # required devices
    turret_motor: lazytalonsrx.LazyTalonSRX

    # required components
    vision: vision.Vision
    imu: lazypigeonimu.LazyPigeonIMU

    class _Mode(Enum):
        Idle = 0
        Output = 1
        Heading = 2

    def __init__(self):
        self.mode = self._Mode.Idle
        self.desired_output = 0
        self.desired_heading = 0
        self.nt = NetworkTables.getTable(f"/components/turret")

    def setup(self):
        # self.turret_motor.setStatusFramePeriod(
        #     lazytalonsrx.LazyTalonSRX.StatusFrame.Status_2_Feedback0,
        #     self.STATUS_FRAME,
        #     self.TIMEOUT,
        # )
        self.turret_motor.setSoftMin(self.SOFT_MIN)
        self.turret_motor.setSoftMax(self.SOFT_MAX)

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
        self.turret_motor.zero()

    def on_disable(self):
        self.stop()

    def stop(self) -> None:
        """Stop the turret."""
        self.mode = self._Mode.Idle

    def _isWithinSoftLimits(self, heading: float) -> bool:
        """Is the given heading within the soft limits."""
        return self.SOFT_MIN <= heading and heading <= self.SOFT_MAX

    def isWithinSoftLimits(self) -> bool:
        """Is the turret heading within the soft limits."""
        heading = self.getHeading()
        return self._isWithinSoftLimits(heading)

    def isReady(self) -> bool:
        """Is the turret ready to shoot balls."""
        return (self.mode == self._Mode.Heading) and abs(
            self.desired_heading - self.getHeading()
        ) <= self.HEADING_TOLERANCE

    def getHeading(self) -> float:
        """Get the heading of the turret."""
        return self.turret_motor.getPosition() * self.OUTPUT_PER_INPUT

    def setOutput(self, output: float) -> None:
        """Set the percent output of the turret."""
        self.mode = self._Mode.Output
        self.desired_output = output

    def setRelativeHeading(self, heading: float) -> None:
        """Set the heading of the turret relative to the current heading."""
        heading += self.getHeading()
        self.setAbsoluteHeading(heading)

    def setAbsoluteHeading(self, heading: float) -> None:
        """Set the absolute heading of the turret."""
        self.mode = self._Mode.Heading
        self.desired_heading = heading

    def setAllocentricHeading(self, heading: float) -> None:
        """Set the allocentric (relative to field) heading of the turret."""
        self.mode = self._Mode.Heading
        self.desired_heading = heading - self.imu.getHeadingInRange()

    def _checkLimitSwitches(self) -> None:
        """Reset turret heading if limit switch triggered."""
        pass
        # if self.turret_motor.isFwdLimitSwitchClosed():
        #     self.turret_motor.zero(self.SOFT_MAX)
        # if self.turret_motor.isRevLimitSwitchClosed():
        #     self.turret_motor.zero(self.SOFT_MIN)

    def _setHeading(self, heading: float):
        """Set the motor position setpoint."""
        heading = units.angle_range(heading)
        if self._isWithinSoftLimits(heading):
            # TODO use regular PID?
            self.turret_motor.setMotionMagicPosition(heading * self.INPUT_PER_OUTPUT)
        else:
            logging.warning(
                f"Trying to set turret at {heading} which is outside of soft limits"
            )

    def _setOutput(self, output: float):
        if self.getHeading() <= self.SOFT_MIN and output <= 0 or self.getHeading() >= self.SOFT_MAX and output >= 0:
            self.turret_motor.set(0)
        else:
            self.turret_motor.set(output)

    def updateNetworkTables(self) -> None:
        """Update network table values related to component."""
        self.nt.putValue("heading", self.getHeading() * units.degrees_per_radian)
        self.nt.putValue(
            "is_at_heading", self.mode == self._Mode.Heading and self.isReady()
        )
        self.nt.putValue(
            "heading_error",
            self.turret_motor.getError() * units.degrees_per_radian
            if self.mode == self._Mode.Heading
            else 0,
        )

    def execute(self):
        self._checkLimitSwitches()
        if self.mode == self._Mode.Idle:
            self.turret_motor.set(0)
        elif self.mode == self._Mode.Output:
            self.turret_motor.setOutput(self.desired_output)
        elif self.mode == self._Mode.Heading:
            self._setHeading(self.desired_heading)

        self.updateNetworkTables()
