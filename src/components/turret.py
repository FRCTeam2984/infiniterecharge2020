from enum import Enum

from networktables import NetworkTables

from components import vision
from controls import pidf
from utils import lazypigeonimu, lazytalonsrx, units
from magicbot import tunable

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
    TURRET_KP = 1 # 1
    TURRET_KI = 0.001 # 0.001
    TURRET_KD = 0.008 # 0.008
    TURRET_KF = 0.08 # 0.08
    TURRET_IZONE = 0

    HEADING_TOLERANCE = 0.5 * units.radians_per_degree

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
        self.turret_motor.setStatusFramePeriod(
            lazytalonsrx.LazyTalonSRX.StatusFrame.Status_2_Feedback0,
            self.STATUS_FRAME,
            self.TIMEOUT,
        )
        self.turret_motor.setBrakeMode()
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

        self.position_pidf = pidf.PIDF(
            self.TURRET_KP, self.TURRET_KI, self.TURRET_KD, self.TURRET_KF,
        )

    def on_enable(self):
        self.turret_motor.zero()
        self.position_pidf = pidf.PIDF(
            self.TURRET_KP, self.TURRET_KI, self.TURRET_KD, self.TURRET_KF,
        )

    def on_disable(self):
        self.stop()

    def stop(self) -> None:
        """Stop the turret."""
        self.mode = self._Mode.Idle

    def isReady(self) -> bool:
        """Is the turret ready to shoot balls."""
        return (self.mode == self._Mode.Heading) and abs(
            self.desired_heading - self.getHeading()
        ) <= self.HEADING_TOLERANCE

    def isMoving(self):
        return self.mode == self._Mode.Heading or self.mode == self._Mode.Output

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
        self.position_pidf.setSetpoint(self.desired_heading)

    def updateNetworkTables(self) -> None:
        """Update network table values related to component."""
        self.nt.putNumber("heading", self.getHeading() * units.degrees_per_radian)
        self.nt.putNumber(
            "heading_error",
            (self.getHeading() - self.desired_heading) * units.degrees_per_radian,
        )
        self.nt.putNumber(
            "desired_heading", self.desired_heading * units.degrees_per_radian
        )
        self.nt.putNumber("desired_output", self.desired_output)
        self.nt.putNumber(
            "is_at_heading", self.mode == self._Mode.Heading and self.isReady()
        )
        self.nt.putBoolean("is_ready", self.isReady())
        self.nt.putBoolean("is_aligning", self.mode == self._Mode.Heading)

    # def _setOutput(self, output):
    #     self.isSo
    def execute(self):
        # self._checkLimitSwitches()
        if self.mode == self._Mode.Idle:
            self.turret_motor.set(0)
        elif self.mode == self._Mode.Output:
            self.turret_motor.setOutput(self.desired_output)
        elif self.mode == self._Mode.Heading:
            self.desired_output = self.position_pidf.update(self.getHeading(), 0.02)
            self.turret_motor.setOutput(self.desired_output)
        self.updateNetworkTables()
