import numpy as np
import rev
from magicbot import tunable
from networktables import NetworkTables
from wpilib import controller

from utils import units


class Flywheel:

    # physical constants
    GEAR_RATIO = 30 / 56
    INPUTS_PER_OUTPUT = 1
    OUTPUTS_PER_INPUT = 1

    # motor config
    INVERTED = True
    CLOSED_LOOP_RAMP = 2.5
    OPEN_LOOP_RAMP = 2.5

    # motor coefs
    FLYWHEEL_KS = 0.491  # V
    FLYWHEEL_KV = 0.002183  # V / (rpm)
    FLYWHEEL_KA = 0.00112  # V / (rpm / s)

    # flywheel pidf gains
    FLYWHEEL_KP = tunable(0)
    FLYWHEEL_KI = tunable(0.00000025)
    FLYWHEEL_KD = tunable(0)
    FLYWHEEL_KF = tunable(0)
    FLYWHEEL_IZONE = tunable(300)

    # percent of setpoint
    RPM_TOLERANCE = 0.05

    # TODO remove
    DESIRED_RPM = tunable(0)

    DISTANCES = (
        np.array((120, 132, 144, 156, 168, 180, 192, 204, 216, 228))
        * units.meters_per_inch
    )
    RPMS = (2430, 2460, 2480, 2530, 2610, 2620, 2670, 2700, 2740, 2760)
    ACCURACY = (1, 1, 1, 1, 0.5, 0.5, 0.5, 0, 0, 0)

    # required devices
    flywheel_motor_left: rev.CANSparkMax

    def __init__(self):
        self.is_spinning = False
        self.desired_rpm = 0
        self.desired_acceleration = 0
        self.feedforward = 0

    def setup(self):
        self.flywheel_motor_left.setInverted(self.INVERTED)
        self.flywheel_motor_left.setOpenLoopRampRate(self.OPEN_LOOP_RAMP)
        self.flywheel_motor_left.setClosedLoopRampRate(self.CLOSED_LOOP_RAMP)

        self.nt = NetworkTables.getTable(
            f"/components/{self.__class__.__name__.lower()}"
        )
        self.encoder = self.flywheel_motor_left.getEncoder()
        self.flywheel_pid = self.flywheel_motor_left.getPIDController()
        self.flywheel_pid.setP(self.FLYWHEEL_KP)
        self.flywheel_pid.setI(self.FLYWHEEL_KI)
        self.flywheel_pid.setD(self.FLYWHEEL_KD)
        self.flywheel_pid.setFF(self.FLYWHEEL_KF)
        self.flywheel_pid.setIZone(self.FLYWHEEL_IZONE)

        self.flywheel_characterization = controller.SimpleMotorFeedforwardMeters(
            self.FLYWHEEL_KS, self.FLYWHEEL_KV, self.FLYWHEEL_KA,
        )

    def on_enable(self):
        self.flywheel_pid.setP(self.FLYWHEEL_KP)
        self.flywheel_pid.setI(self.FLYWHEEL_KI)
        self.flywheel_pid.setD(self.FLYWHEEL_KD)
        self.flywheel_pid.setFF(self.FLYWHEEL_KF)
        self.flywheel_pid.setIZone(self.FLYWHEEL_IZONE)

    def on_disable(self):
        self.stop()

    def setRPM(self, rpm) -> None:
        """Set the rpm of the flywheel."""
        self.is_spinning = True
        self.desired_rpm = rpm

    def setDistance(self, distance):
        desired_rpm = np.interp(distance, self.DISTANCES, self.RPMS)
        self.setRPM(desired_rpm)

    def stop(self) -> None:
        """Stop the flywheel."""
        self.is_spinning = False
        self.desired_rpm = 0

    def isAtSetpoint(self) -> bool:
        """Is the flywheel at the desired speed."""
        return abs(self.desired_rpm - self.encoder.getVelocity()) <= (
            self.desired_rpm * self.RPM_TOLERANCE
        )

    def isReady(self) -> bool:
        """Is the flywheel ready for a ball."""
        return self.isAtSetpoint()

    def _calculateFF(self):
        """Calculate the feedforward voltage given current the current state."""
        self.desired_acceleration = self.desired_rpm - self.encoder.getVelocity()
        self.feedforward = self.flywheel_characterization.calculate(
            self.desired_rpm, self.desired_acceleration
        )

    def updateNetworkTables(self):
        """Update network table values related to component."""
        self.nt.putNumber("desired_rpm", self.desired_rpm)
        self.nt.putNumber("desired_accel", self.desired_acceleration)
        self.nt.putNumber("feedforward", self.feedforward)
        self.nt.putNumber(
            "actual_rpm", self.encoder.getVelocity() * self.OUTPUTS_PER_INPUT
        )

    def execute(self):
        # TODO remove
        # self.desired_rpm = self.DESIRED_RPM

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
            self.flywheel_motor_left.set(0.0)

        self.updateNetworkTables()
