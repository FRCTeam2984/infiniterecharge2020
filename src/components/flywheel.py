import numpy as np
import rev
from networktables import NetworkTables
from wpilib import controller

from utils import units


class Flywheel:

    # physical constants
    GEAR_RATIO = 30 / 56
    INPUTS_PER_OUTPUT = GEAR_RATIO
    OUTPUTS_PER_INPUT = 1 / GEAR_RATIO

    # motor config
    INVERTED = True

    CLOSED_LOOP_RAMP = 0
    OPEN_LOOP_RAMP = 0

    CURRENT_LIMIT = 29

    # motor coefs
    # TODO remove ka?
    FLYWHEEL_KS = 0.491  # V
    FLYWHEEL_KV = 0.00195  # 0.002183  # V / (rpm)
    FLYWHEEL_KA = 0  # 0.00112  # V / (rpm / s)

    # flywheel pidf gains
    # TODO tune
    FLYWHEEL_KP = 0.0008
    FLYWHEEL_KI = 0
    FLYWHEEL_KD = 0
    FLYWHEEL_KF = 0
    FLYWHEEL_IZONE = 0

    # percent of setpoint
    RPM_TOLERANCE = 0.18

    DISTANCES = (
        np.array((10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 27, 30, 32))
        * units.meters_per_foot
    )
    RPMS = (
        4540,  # 10
        4590,  # 11
        4630,  # 12
        4720,  # 13
        4870,  # 14
        4890,  # 15
        4980,  # 16
        5040,  # 17
        5110,  # 18
        5200,  # 21
        5400,  # 24
        5700,  # 27
        5900,  # 30
        6200,  # 32
    )
    # ACCURACY = (1, 1, 1, 1, 0.75, 0.75, 0.75, 0.5, 0.5, 0.5)

    SLOPE = 231.28
    INTERCEPT = 3817

    REV_RPM = 5000

    # required devices
    flywheel_motor: rev.CANSparkMax

    def __init__(self):
        self.is_spinning = False
        self.desired_rpm = 0
        self.desired_acceleration = 0
        self.feedforward = 0

    def setup(self):
        self.nt = NetworkTables.getTable(f"/components/flywheel")
        self.flywheel_motor.setInverted(self.INVERTED)
        self.flywheel_motor.setOpenLoopRampRate(self.OPEN_LOOP_RAMP)
        self.flywheel_motor.setClosedLoopRampRate(self.CLOSED_LOOP_RAMP)
        self.flywheel_motor.setSmartCurrentLimit(self.CURRENT_LIMIT)

        self.encoder = self.flywheel_motor.getEncoder()
        self.flywheel_pid = self.flywheel_motor.getPIDController()
        self.flywheel_pid.setP(self.FLYWHEEL_KP)
        self.flywheel_pid.setI(self.FLYWHEEL_KI)
        self.flywheel_pid.setD(self.FLYWHEEL_KD)
        self.flywheel_pid.setFF(self.FLYWHEEL_KF)
        self.flywheel_pid.setIZone(self.FLYWHEEL_IZONE)

        self.flywheel_characterization = controller.SimpleMotorFeedforwardMeters(
            self.FLYWHEEL_KS, self.FLYWHEEL_KV, self.FLYWHEEL_KA,
        )
        # self.flywheel_motor.burnFlash()

    def on_enable(self):
        # TODO remove
        self.flywheel_pid.setP(self.FLYWHEEL_KP)
        self.flywheel_pid.setI(self.FLYWHEEL_KI)
        self.flywheel_pid.setD(self.FLYWHEEL_KD)
        self.flywheel_pid.setFF(self.FLYWHEEL_KF)
        self.flywheel_pid.setIZone(self.FLYWHEEL_IZONE)
        self.flywheel_characterization = controller.SimpleMotorFeedforwardMeters(
            self.FLYWHEEL_KS, self.FLYWHEEL_KV, self.FLYWHEEL_KA,
        )

    def on_disable(self):
        self.stop()

    def setRPM(self, rpm) -> None:
        """Set the rpm of the flywheel."""
        self.is_spinning = True
        self.desired_rpm = rpm

    def setDistance(self, distance):
        """Interpolate the RPM of the flywheel from the distance to the target."""
        # desired_rpm = np.interp(distance, self.DISTANCES, self.RPMS)
        desired_rpm = self.SLOPE * distance + self.INTERCEPT
        self.setRPM(desired_rpm)

    def revUp(self) -> None:
        self.is_spinning = True
        self.desired_rpm = self.REV_RPM

    def stop(self) -> None:
        """Stop the flywheel."""
        self.is_spinning = False
        self.desired_rpm = 0

    def isAtSetpoint(self) -> bool:
        """Is the flywheel at the desired speed."""
        return abs(self.desired_rpm - self.getVelocity()) <= (
            self.desired_rpm * self.RPM_TOLERANCE
        )

    def isReady(self) -> bool:
        """Is the flywheel ready for a ball."""
        return self.isAtSetpoint()

    def getVelocity(self):
        return self.encoder.getVelocity() * self.OUTPUTS_PER_INPUT

    def _calculateFF(self):
        """Calculate the feedforward voltage given current the current state."""
        self.desired_acceleration = self.desired_rpm - self.getVelocity()

        desired_rpm = self.desired_rpm * self.INPUTS_PER_OUTPUT
        desired_acceleration = self.desired_acceleration * self.INPUTS_PER_OUTPUT

        self.feedforward = self.flywheel_characterization.calculate(
            desired_rpm, desired_acceleration
        )

    def updateNetworkTables(self):
        """Update network table values related to component."""
        self.nt.putNumber("actual_rpm", self.getVelocity())
        self.nt.putBoolean("is_spinning", self.is_spinning)
        self.nt.putNumber("desired_rpm", self.desired_rpm)
        self.nt.putNumber("error_rpm", self.getVelocity() - self.desired_rpm)
        self.nt.putNumber("desired_accel", self.desired_acceleration)
        self.nt.putNumber("feedforward", self.feedforward)
        self.nt.putBoolean("is_ready", self.isReady())

    def execute(self):
        # calculate feedword terms
        self._calculateFF()

        if self.is_spinning:
            # set the flywheel at the desired rpm
            self.flywheel_pid.setReference(
                self.desired_rpm * self.INPUTS_PER_OUTPUT,
                rev.ControlType.kVelocity,
                0,
                self.feedforward,
            )
        else:
            # stop the flywheel
            self.flywheel_motor.stopMotor()

        self.updateNetworkTables()
