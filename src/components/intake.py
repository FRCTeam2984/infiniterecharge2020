from utils import lazytalonsrx


class Intake:

    # speed to run intake
    INTAKE_SPEED = 1.0

    # required devices
    intake_motor: lazytalonsrx.LazyTalonSRX

    def __init__(self):
        self.is_intaking = False

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop()

    def intake(self) -> None:
        """Start intaking balls."""
        self.is_intaking = True

    def stop(self) -> None:
        """Stop intake motor."""
        self.is_intaking = False

    def execute(self):
        if self.is_intaking:
            self.intake_motor.setOutput(self.INTAKE_SPEED)
        else:
            self.intake_motor.setOutput(0.0)
