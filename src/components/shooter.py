import logging
from enum import Enum
import rev
from utils import units
import numpy as np
from magicbot import tunable


class Shooter:

    SHOOT_SPEED = 4000

    WHEEL_RADIUS = 2 * units.meters_per_inch
    WHEEL_CIRCUMFERENCE = 2 * np.pi * WHEEL_RADIUS
    # RPM_PER_MPS = 60 / WHEEL_CIRCUMFERENCE
    # MPS_PER_RPM = WHEEL_CIRCUMFERENCE / 60

    SHOOTER_KP = tunable(0)
    SHOOTER_KI = tunable(0)
    SHOOTER_KD = tunable(0)
    SHOOTER_KF = tunable(0)

    # percent of setpoint
    RPM_TOLERANCE = 0.05

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

    def shoot(self) -> None:
        """Start spinning the shooter."""
        self.is_shooting = True
        self.rpm = self.SHOOT_SPEED

    def stop(self) -> None:
        """Stop the shooter."""
        self.is_shooting = False
        self.rpm = 0

    def isAtSetpoint(self):
        return abs(self.rpm - self.encoder.getVelocity()) <= (
            self.rpm * self.RPM_TOLERANCE
        )

    def isReady(self):
        return self.isAtSetpoint()

    def execute(self):
        self.shooter_pid.setReference(self.rpm, rev.ControlType.kVelocity)
