import logging
from enum import Enum
import rev
from utils import units
import numpy as np
from magicbot import tunable, feedback


class Shooter:


    # shooter physical constants
    WHEEL_RADIUS = 2 * units.meters_per_inch
    WHEEL_CIRCUMFERENCE = 2 * np.pi * WHEEL_RADIUS

    # speed to shoot balls
    SHOOT_SPEED = 4000

    # shooter pidf gains
    SHOOTER_KP = tunable(0)
    SHOOTER_KI = tunable(0)
    SHOOTER_KD = tunable(0)
    SHOOTER_KF = tunable(0)

    # percent of setpoint
    RPM_TOLERANCE = 0.05

    # required devices
    shooter_motor_left: rev.CANSparkMax

    def __init__(self):
        self.is_shooting = False
        self.rpm = 0

    def setup(self):
        self.encoder = self.shooter_motor_left.getEncoder()
        self.shooter_pid = self.shooter_motor_left.getPIDController()
        self.shooter_pid.setP(self.SHOOTER_KP)
        self.shooter_pid.setI(self.SHOOTER_KI)
        self.shooter_pid.setD(self.SHOOTER_KD)
        self.shooter_pid.setFF(self.SHOOTER_KF)
        self.shooter_pid.setIZone(0)

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop()

    def shoot(self) -> None:
        """Start spinning the shooter."""
        self.is_shooting = True
        self.rpm = self.SHOOT_SPEED

    def stop(self) -> None:
        """Stop the shooter."""
        self.is_shooting = False
        self.rpm = 0

    def isAtSetpoint(self) -> bool:
        """Is the shooter at the desired speed."""
        return abs(self.rpm - self.encoder.getVelocity()) <= (
            self.rpm * self.RPM_TOLERANCE
        )

    def isReady(self) -> bool:
        """Is the shooter ready for a ball."""
        return self.isAtSetpoint()

    @feedback
    def is_ready(self):
        return self.isReady()

    def execute(self):
        if self.is_shooting:
            self.shooter_pid.setReference(self.rpm, rev.ControlType.kVelocity)
        else:
            self.shooter_motor_left.set(0.0)
