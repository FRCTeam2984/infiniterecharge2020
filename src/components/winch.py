from magicbot import tunable

from utils import lazytalonsrx


class Winch:

    # motor speeds
    WIND_SPEED = tunable(1.0)
    UNWIND_SPEED = tunable(1.0)

    # motor config
    INVERTED = False

    # required devices
    climb_winch_master: lazytalonsrx.LazyTalonSRX

    def __init__(self):
        self.is_hoisting = False
        self.desired_output = 0

    def setup(self):
        self.climb_winch_master.setInverted(self.INVERTED)

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop()

    def wind(self) -> None:
        """Hoist up robot."""
        self.is_hoisting = True
        self.desired_output = self.WIND_SPEED

    def unwind(self) -> None:
        """Lower down robot."""
        self.is_hoisting = True
        self.desired_output = -self.UNWIND_SPEED

    def stop(self) -> None:
        """Stop winch."""
        self.is_hoisting = False
        self.desired_output = 0

    def execute(self):
        if self.is_hoisting:
            self.climb_winch_master.setOutput(self.desired_output)
        else:
            self.climb_winch_master.setOutput(0)
