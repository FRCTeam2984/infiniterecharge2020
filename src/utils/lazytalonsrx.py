import logging

import numpy as np

import ctre


class EncoderType:
    Quad = ctre.FeedbackDevice.QuadEncoder
    Integrated = ctre.FeedbackDevice.IntegratedSensor
    CTREMag = ctre.FeedbackDevice.CTRE_MagEncoder_Relative


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


CTREMag = EncoderConfig(EncoderType.CTREMag, 4096)
FalconEncoder = EncoderConfig(EncoderType.Integrated, 2048)


class LazyTalonSRX(ctre.WPI_TalonSRX):
    """A wraper for the ctre.WPI_TalonSRX to simplfy configuration and getting/setting values."""

    TIMEOUT = 10

    ControlMode = ctre.ControlMode
    DemandType = ctre.DemandType
    StatusFrame = ctre.StatusFrameEnhanced
    NeutralMode = ctre.NeutralMode

    def __init__(self, id: int):
        super().__init__(id)
        self.no_encoder_warning = f"No encoder connected to Talon {id}"
        self.no_closed_loop_warning = f"Talon {id} not in closed loop mode"

    def initialize(self, name: str = None) -> None:
        """Initialize the motors (enable the encoder, set invert status, set voltage limits)."""
        self.encoder = False
        if name != None:
            self.setName(name)

    def setEncoderConfig(self, config: EncoderConfig, phase: bool) -> None:
        self.encoder = True
        self.encoder_config = config
        self.configSelectedFeedbackSensor(config.type, 0, self.TIMEOUT)
        self.setSensorPhase(phase)

    def setPIDF(self, slot: int, kp: float, ki: float, kd: float, kf: float) -> None:
        """Initialize the PIDF controller."""
        self.selectProfileSlot(slot, self.TIMEOUT)
        self.config_kP(slot, kp, self.TIMEOUT)
        self.config_kI(slot, ki, self.TIMEOUT)
        self.config_kD(slot, kd, self.TIMEOUT)
        self.config_kF(slot, kf, self.TIMEOUT)

    def setIZone(self, slot: int, izone: float) -> None:
        """Set the izone of the PIDF controller."""
        self.config_IntegralZone(
            slot, int(izone * self.encoder_config.counts_per_radian), self.TIMEOUT
        )

    def setBrakeMode(self):
        self.setNeutralMode(self.NeutralMode.Brake)

    def setCoastMode(self):
        self.setNeutralMode(self.NeutralMode.Coast)

    def setSoftMax(self, limit: float):
        self.configForwardSoftLimitThreshold(
            int(limit * self.encoder_config.counts_per_radian)
        )
        self.configForwardSoftLimitEnable(True)

    def setSoftMin(self, limit: float):
        self.configReverseSoftLimitThreshold(
            int(limit * self.encoder_config.counts_per_radian)
        )
        self.configReverseSoftLimitEnable(True)

    def setMotionMagicConfig(self, vel: float, accel: float) -> None:
        self.configMotionCruiseVelocity(
            int(vel * self.encoder_config.counts_per_radian / 10), self.TIMEOUT
        )
        self.configMotionAcceleration(
            int(accel * self.encoder_config.counts_per_radian / 10), self.TIMEOUT
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

    def zero(self, pos: float = 0) -> None:
        """Zero the encoder if it exists."""
        if self.encoder:
            self.setSelectedSensorPosition(
                int(pos * self.encoder_config.counts_per_radian), 0, self.TIMEOUT
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

    def _isClosedLoop(self) -> bool:
        return self.getControlMode() in (
            self.ControlMode.Velocity,
            self.ControlMode.Position,
            self.ControlMode.MotionMagic,
        )
