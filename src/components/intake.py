import logging

from utils import lazytalonsrx


class Intake:

    INTAKE_SPEED = 1.0

    intake_motor: lazytalonsrx.LazyTalonSRX

    def __init__(self):
        self.intaking = False

    def on_enable(self):
        pass

    def intake(self) -> None:
        self.intaking = True

    def stop(self) -> None:
        self.intaking = False

    def execute(self):
        if self.intaking:
            self.intake_motor.setOutput(self.INTAKE_SPEED)
        else:
            self.intake_motor.setOutput(0.0)
