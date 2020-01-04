import logging

import ctre
from networktables import NetworkTables
from enum import IntEnum
import numpy as np


class EncoderType(IntEnum):
    Quad = ctre.FeedbackDevice.QuadEncoder


class EncoderConfig:
    def __init__(self, _type: EncoderType, cpr: int):
        self.type = _type
        self.cpr = cpr

    @property
    def radians_per_count(self):
        return (2 * np.pi) / self.cpr

    @property
    def counts_per_radian(self):
        return self.cpr / (2 * np.pi)


CTREMag = EncoderConfig(EncoderType.Quad, 4096)


class LazyTalonSRX(ctre.WPI_TalonSRX):
    """A wraper for the ctre.WPI_TalonSRX to simplfy configuration and getting/setting values."""

    MotorDash = NetworkTables.getTable("SmartDashboard").getSubTable("TalonSRX")
    ControlMode = ctre.WPI_TalonSRX.ControlMode
    DemandType = ctre.WPI_TalonSRX.DemandType

    def __init__(self, id: int):
        super().__init__(id)

    def initialize(self, name: str = None) -> None:
        """Initialize the motors (enable the encoder, set invert status, set voltage limits)."""
        self.encoder = False
        if name != None:
            self.setName(name)
        self.no_encoder_warning = f"No encoder connected to {self.name}"
        self.no_closed_loop_warning = f"{self.name} not in closed loop mode"

    def setEncoderConfig(self, config: EncoderConfig, phase: bool) -> None:
        self.encoder = True
        self.encoder_config = config
        self.configSelectedFeedbackSensor(config.type, 0, timeoutMs=10)
        self.setSensorPhase(phase)

    def setPIDF(self, slot: int, kp: float, ki: float, kd: float, kf: float) -> None:
        """Initialize the PIDF controller."""
        self.selectProfileSlot(slot, 0)
        self.config_kP(slot, kp, 0)
        self.config_kI(slot, ki, 0)
        self.config_kD(slot, kd, 0)
        self.config_kF(slot, kf, 0)

    def setMotionMagicConfig(self, vel: float, accel: float) -> None:
        self.configMotionAcceleration(
            int(accel * self.encoder_config.counts_per_radian), 0
        )
        self.configMotionCruiseVelocity(
            int(vel * self.encoder_config.counts_per_radian), 0
        )

    def setOutput(self, signal: float, max_signal: float = 1) -> None:
        """Set the percent output of the motor."""
        signal = np.clip(signal, -max_signal, max_signal)
        self.set(self.ControlMode.PercentOutput, signal)

    def setPosition(self, pos: float) -> None:
        """Set the position of the motor."""
        self.set(self.ControlMode.Position, pos * self.encoder_config.counts_per_radian)

    def setVelocity(self, vel: float, ff: float = 0) -> None:
        """Set the velocity of the motor."""
        self.set(
            self.ControlMode.Velocity,
            vel * self.encoder_config.counts_per_radian / 10,
            self.DemandType.ArbitraryFeedForward,
            ff,
        )

    def setMotionMagicPosition(self, pos: float) -> None:
        """Set the position of the motor using motion magic."""
        self.set(
            self.ControlMode.MotionMagic, pos * self.encoder_config.counts_per_radian
        )

    def zero(self, pos: int = 0) -> None:
        """Zero the encoder if it exists."""
        if self.encoder:
            self.setSelectedSensorPosition(
                pos * self.encoder_config.counts_per_radian, 0, 0
            )
        else:
            logging.warning(self.no_encoder_warning)

    def getPosition(self) -> int:
        """Get the encoder position if it exists."""
        if self.encoder:
            return (
                self.getSelectedSensorPosition(0)
                * self.encoder_config.radians_per_count
            )
        else:
            logging.warning(self.no_encoder_warning)
            return 0

    def getVelocity(self) -> int:
        """Get the encoder velocity if it exists."""
        if self.encoder:
            return (
                self.getSelectedSensorVelocity(0)
                * self.encoder_config.radians_per_count
                * 10
            )
        else:
            logging.warning(self.no_encoder_warning)
            return 0

    def getError(self) -> int:
        """Get the closed loop error if in closed loop mode."""
        if self._isClosedLoop():
            return self.getClosedLoopError(0)
        else:
            logging.warning(self.no_closed_loop_warning)
            return 0

    def getTarget(self) -> int:
        """Get the closed loop target if in closed loop mode."""
        if self._isClosedLoop():
            return self.getClosedLoopTarget(0)
        else:
            logging.warning(self.no_closed_loop_warning)
            return 0

    def outputToDashboard(self) -> None:
        pass
        self.MotorDash.putNumber(
            f"{self.name} Percent Output", self.getMotorOutputPercent()
        )
        if self.encoder:
            self.MotorDash.putNumber(f"{self.name} Position", self.getPosition())
            if self._isClosedLoop():
                self.MotorDash.putNumber(
                    f"{self.name} PIDF Target", self.getClosedLoopTarget(0)
                )
                self.MotorDash.putNumber(
                    f"{self.name} PIDF Error", self.getClosedLoopError(0)
                )

    def _isClosedLoop(self) -> bool:
        return self.getControlMode() in (
            ctre.WPI_TalonSRX.ControlMode.Velocity,
            ctre.WPI_TalonSRX.ControlMode.Position,
            ctre.WPI_TalonSRX.ControlMode.MotionMagic,
        )
