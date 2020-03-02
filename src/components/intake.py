import wpilib
from magicbot import tunable
from networktables import NetworkTables

from utils import lazytalonsrx


class Intake:

    # speed to run intake
    INTAKE_SPEED = tunable(1)
    OUTTAKE_SPEED = tunable(-1)

    # required devices
    intake_motor: lazytalonsrx.LazyTalonSRX
    intake_limit: wpilib.DigitalInput

    def __init__(self):
        self.is_taking = False
        self.desired_output = 0

    def setup(self):
        self.nt = NetworkTables.getTable(f"/components/intake")

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop()

    def intake(self) -> None:
        """Start intaking balls."""
        self.is_taking = True
        self.desired_output = self.INTAKE_SPEED

    def outtake(self) -> None:
        """Start outtaking balls."""
        self.is_taking = True
        self.desired_output = self.OUTTAKE_SPEED

    def stop(self) -> None:
        """Stop intake motor."""
        self.is_taking = False
        self.desired_output = 0

    def hasBall(self):
        return not self.intake_limit.get()

    def updateNetworkTables(self):
        """Update network table values related to component."""
        self.nt.putNumber("stator_current", self.intake_motor.getStatorCurrent())

    def execute(self):
        if self.is_taking:
            self.intake_motor.setOutput(self.desired_output)
        else:
            self.intake_motor.setOutput(0)
        self.updateNetworkTables()
