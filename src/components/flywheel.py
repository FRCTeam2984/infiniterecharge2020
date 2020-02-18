import logging
from enum import Enum
import rev
from utils import units
import numpy as np
from magicbot import tunable, feedback
from wpilib import controller
from networktables import NetworkTables


class Flywheel:

    # motor config
    INVERTED = True
    # motor coefs
    FLYWHEEL_KS = 0.0574  # V
    FLYWHEEL_KV = 0.131  # V / (rpm)
    FLYWHEEL_KA = 0.0612  # V / (rpm / s)

    # flywheel pidf gains
    FLYWHEEL_KP = tunable(0)
    FLYWHEEL_KI = tunable(0)
    FLYWHEEL_KD = tunable(0)
    FLYWHEEL_KF = tunable(0)
    FLYWHEEL_IZONE = tunable(0)

    # percent of setpoint
    RPM_TOLERANCE = 0.05
    DESIRED_RPM = tunable(0)
    # required devices
    flywheel_motor_left: rev.CANSparkMax

    def __init__(self):
        self.is_spinning = False
        self.desired_rpm = 0
        self.desired_acceleration = 0
        self.feedforward = 0

    def setup(self):
        self.flywheel_motor_left.setInverted(self.INVERTED)

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
        pass

    def on_disable(self):
        self.stop()

    def setRPM(self, rpm) -> None:
        """Set the rpm of the flywheel."""
        self.is_spinning = True
        self.desired_rpm = rpm

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
            self.desired_rpm / 60, self.desired_acceleration / 60
        )

    def updateNetworkTables(self):
        """Update network table values related to component."""
        self.nt.putNumber("desired_rpm", self.desired_rpm)
        self.nt.putNumber("desired_accel", self.desired_rpm)
        self.nt.putNumber("feedforward", self.feedforward)
        self.nt.putNumber("actual_rpm", self.encoder.getVelocity())

    def execute(self):
        # calculate feedword terms
        self._calculateFF()

        if self.is_spinning:
            # set the flywheel at the desired rpm
            self.flywheel_pid.setReference(
                self.desired_rpm, rev.ControlType.kVelocity, 0, self.feedforward
            )
        else:
            # stop the flywheel
            self.flywheel_motor_left.set(0.0)

        self.updateNetworkTables()
