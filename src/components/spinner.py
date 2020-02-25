from enum import IntEnum

from utils import lazytalonsrx


class Colors(IntEnum):
    UNKOWN = 0
    RED = 1
    YELLOW = 2
    BLUE = 3
    FREE = 4


class Spinner:

    # speed to run spinner
    SPIN_SPEED = 0.5

    # required devices
    spinner_motor: lazytalonsrx.LazyTalonSRX

    def __init__(self):
        self.is_spinning = False
        self.color = Colors.UNKOWN

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop()

    def setOutput(self, output: float) -> None:
        self.is_spinning = True
        self.desired_output = output

    def forward(self) -> None:
        """Start spinning spinner forward."""
        self.setOutput(self.SPIN_SPEED)

    def backward(self) -> None:
        """Start spinning spinner backward."""
        self.setOutput(-self.SPIN_SPEED)

    def stop(self) -> None:
        """Stop spinner motor."""
        self.is_spinning = False
        self.desired_output = 0

    def getColor(self):
        return self.color

    def execute(self):
        if self.is_spinning:
            self.spinner_motor.setOutput(self.desired_output)
        else:
            self.spinner_motor.setOutput(0)

        # TODO handle color
        self.color = Colors.UNKOWN
