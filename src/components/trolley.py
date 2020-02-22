from utils import lazytalonsrx


class Trolley:

    # motor config
    INVERTED = False

    # required devices
    trolley_motor: lazytalonsrx.LazyTalonSRX

    def __init__(self):
        self.is_moving = False
        self.desired_output = 0

    def setup(self):
        self.trolley_motor.setInverted(self.INVERTED)

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop()

    def setOutput(self, output: float) -> None:
        """Set percent output of the slider motor."""
        self.is_moving = True
        self.desired_output = output

    def stop(self) -> None:
        """Stop trolley motor."""
        self.is_moving = False
        self.desired_output = 0

    def execute(self):
        if self.is_moving:
            self.trolley_motor.setOutput(self.desired_output)
        else:
            self.trolley_motor.setOutput(0)
