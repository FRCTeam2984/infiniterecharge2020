import numpy as np
from networktables import NetworkTables

import rev
from magicbot import tunable
from utils import units
from wpilib import controller


class Flywheel:

    # physical constants
    GEAR_RATIO = 30 / 56
    INPUTS_PER_OUTPUT = GEAR_RATIO
    OUTPUTS_PER_INPUT = 1 / GEAR_RATIO

    # motor config
    INVERTED = True
    CLOSED_LOOP_RAMP = 2.5
    OPEN_LOOP_RAMP = 2.5

    # motor coefs
    # TODO remove ka?
    FLYWHEEL_KS = 0.491  # V
    FLYWHEEL_KV = 0.002183  # V / (rpm)
    FLYWHEEL_KA = 0  # 0.00112  # V / (rpm / s)

    # flywheel pidf gains
    # TODO tune
    FLYWHEEL_KP = tunable(0)
    FLYWHEEL_KI = tunable(0)  # tunable(0.00000025)
    FLYWHEEL_KD = tunable(0)
    FLYWHEEL_KF = tunable(0)
    FLYWHEEL_IZONE = tunable(0)  # tunable(300)

    # percent of setpoint
    RPM_TOLERANCE = 0.05

    # TODO remove
    DESIRED_RPM = tunable(0)

    DISTANCES = (
        np.array((10, 11, 12, 13, 14, 15, 16, 17, 18, 19)) * units.meters_per_foot
    )
    RPMS = (
        np.array((2430, 2460, 2480, 2530, 2610, 2620, 2670, 2700, 2740, 2760))
        * OUTPUTS_PER_INPUT
    )
    # (4540, 4590, 4630, 4720, 4870, 4890, 4980, 5040, 5110, 5150)
    ACCURACY = (1, 1, 1, 1, 0.75, 0.75, 0.75, 0.5, 0.5, 0.5)

    # required devices
    flywheel_master: rev.CANSparkMax

    def __init__(self):
        self.is_spinning = False
        self.desired_rpm = 0
        self.desired_acceleration = 0
        self.feedforward = 0

    def setup(self):
        self.flywheel_master.setInverted(self.INVERTED)
        self.flywheel_master.setOpenLoopRampRate(self.OPEN_LOOP_RAMP)
        self.flywheel_master.setClosedLoopRampRate(self.CLOSED_LOOP_RAMP)

        self.nt = NetworkTables.getTable(
            f"/components/{self.__class__.__name__.lower()}"
        )
        self.encoder = self.flywheel_master.getEncoder()
        self.flywheel_pid = self.flywheel_master.getPIDController()
        self.flywheel_pid.setP(self.FLYWHEEL_KP)
        self.flywheel_pid.setI(self.FLYWHEEL_KI)
        self.flywheel_pid.setD(self.FLYWHEEL_KD)
        self.flywheel_pid.setFF(self.FLYWHEEL_KF)
        self.flywheel_pid.setIZone(self.FLYWHEEL_IZONE)

        self.flywheel_characterization = controller.SimpleMotorFeedforwardMeters(
            self.FLYWHEEL_KS, self.FLYWHEEL_KV, self.FLYWHEEL_KA,
        )

    def on_enable(self):
        # TODO remove
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
        """Interpolate the RPM of the flywheel from the distance to the target."""
        desired_rpm = np.interp(distance, self.DISTANCES, self.RPMS)
        self.setRPM(desired_rpm)

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
        self.nt.putNumber("desired_rpm", self.desired_rpm)
        self.nt.putNumber("desired_accel", self.desired_acceleration)
        self.nt.putNumber("feedforward", self.feedforward)
        self.nt.putNumber("actual_rpm", self.getVelocity())

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
            self.flywheel_master.set(0.0)

        self.updateNetworkTables()
