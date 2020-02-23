from utils import lazytalonsrx


class Slider:

    # speeds to run slider
    EXTEND_SPEED = 0.2
    RETRACT_SPEED = -0.2

    # motor config
    INVERTED = False

    # required devices
    slider_motor: lazytalonsrx.LazyTalonSRX

    def __init__(self):
        self.is_sliding = False
        self.desired_output = 0

    def setup(self):
        self.slider_motor.setInverted(self.INVERTED)

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop()

    def extend(self) -> None:
        """Extend slider."""
        self.is_sliding = True
        self.desired_output = self.EXTEND_SPEED

    def retract(self) -> None:
        """Retract slider."""
        self.is_sliding = True
        self.desired_output = self.RETRACT_SPEED

    def stop(self) -> None:
        """Stop slider motor."""
        self.is_sliding = False
        self.desired_output = 0

    def execute(self):
        if self.is_sliding:
            self.slider_motor.setOutput(self.desired_output)
        else:
            self.slider_motor.setOutput(0)
