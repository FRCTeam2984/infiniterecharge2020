from utils import lazytalonsrx


class Trolley:

    # motor config
    INVERTED = False

    # joystick config
    JOYSTICK_SCALAR = 0.5
    JOYSTICK_DEADBAND = 0.05

    # required devices
    trolley_motor: lazytalonsrx.LazyTalonSRX

    def __init__(self):
        self.is_moving = False
        self.desired_output = 0
        self.is_locked = False

    def setup(self):
        self.trolley_motor.setInverted(self.INVERTED)

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop()

    def setFromJoystick(self, output: float) -> None:
        """Set percent output of the slider motor from joystick input."""
        output *= self.JOYSTICK_SCALAR
        output = 0 if abs(output) <= self.JOYSTICK_DEADBAND else output
        self.setOutput(output)

    def setOutput(self, output: float) -> None:
        """Set percent output of the slider motor."""
        self.is_moving = True
        self.desired_output = output

    def stop(self) -> None:
        """Stop trolley motor."""
        self.is_moving = False
        self.desired_output = 0

    def lock(self, lock: bool):
        self.is_locked = lock
        if self.is_locked:
            self.trolley_motor.setBrakeMode()
        else:
            self.trolley_motor.setCoastMode()

    def isLocked(self):
        return self.is_locked

    def execute(self):
        if self.is_moving:
            self.trolley_motor.setOutput(self.desired_output)
        else:
            self.trolley_motor.setOutput(0)
