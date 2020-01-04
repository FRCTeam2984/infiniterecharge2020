import logging
import wpilib
import ctre
from enum import Enum
from utils import lazytalonsrx, lazypigeonimu


class Chassis:

    dm_l: lazytalonsrx.LazyTalonSRX
    dm_r: lazytalonsrx.LazyTalonSRX
    gyro: lazypigeonimu.LazyPigeonIMU

    class _Mode(Enum):
        Nil = 0
        PercentOutput = 1

    def __init__(self):
        self.signal_l = 0
        self.signal_r = 0
        self.mode = self._Mode.Nil

    def on_enable(self):
        pass

    def checkErrors(self) -> bool:
        """Check and log any errors regarding the component. Returns true if the error is significant enough to warrant a cease of all operations."""
        status = False
        if False:  # dm_l is not connected:
            logging.error(f"{self.dm_l.name} is not connected.")
            status = True
        if False:  # dm_r is not connected:
            logging.error(f"{self.dm_r.name} is not connected.")
            status = True
        if False:  # dm_l current draw is too high:
            logging.error(f"{self.dm_l.name} is drawing too much current.")
            status = True
        if False:  # dm_r current draw is too high:
            logging.error(f"{self.dm_r.name} is drawing too much current.")
            status = True
        return status

    def setFromJoystick(self, throttle, rotation) -> None:
        """Set the output of the motors from joystick values."""
        output_l = throttle - rotation
        output_r = throttle + rotation
        self.setOutput(output_l, output_r)

    def setOutput(self, output_l: float, output_r: float) -> None:
        """Set the output of the motors from percent values."""
        self.mode = self._Mode.PercentOutput
        self.signal_l = output_l
        self.signal_r = output_r

    def execute(self):
        if self.checkErrors():
            logging.error(
                "Chassis has encountered a significant error, ceasing all operations."
            )
            return
        logging.info(f"{self.gyro.getYaw()} and {self.gyro.getYawInRange()}")
        if self.mode == self._Mode.PercentOutput:
            self.dm_l.setOutput(self.signal_l)
            self.dm_r.setOutput(self.signal_r)
