import logging
from enum import Enum
import rev
from utils import units, lazytalonsrx
import numpy as np
import ctre


class Tower:

    # speeds at which to run motors
    LOW_TOWER_SPEED = 0.5
    HIGH_TOWER_SPEED = 0.5

    # required devices
    low_tower_motor: lazytalonsrx.LazyTalonSRX
    high_tower_motor: lazytalonsrx.LazyTalonSRX

    def __init__(self):
        self.is_lifting = False
        self.ball_count = [False, False, False, False]

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop()

    def lift(self) -> None:
        """Start lifting up balls."""
        self.is_lifting = True

    def stop(self) -> None:
        """Stop lifting balls."""
        self.is_lifting = False

    def hasBall(self, index: int) -> bool:
        """Does the tower have a ball at the given index."""
        return self.ball_count[index]

    def isFullyLoaded(self) -> bool:
        """Are 4 balls in the tower."""
        return all(self.ball_count)

    def execute(self):
        # TODO handle tower
        if self.is_lifting:
            self.low_tower_motor.setOutput(self.LOW_TOWER_SPEED)
            self.high_tower_motor.setOutput(self.HIGH_TOWER_SPEED)
        else:
            self.low_tower_motor.setOutput(0.0)
            self.high_tower_motor.setOutput(0.0)
        # TODO track_balls
        if False:  # trigger 0
            self.ball_count[0] = True
        if False:  # trigger 1
            self.ball_count[1] = True
        if False:  # trigger 2
            self.ball_count[2] = True
        if False:  # trigger 3
            self.ball_count[3] = True
