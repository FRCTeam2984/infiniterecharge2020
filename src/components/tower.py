from utils import lazytalonsrx
from magicbot import tunable


class Tower:

    # speeds at which to run motors
    LOW_TOWER_LIFT_SPEED = tunable(0.5)
    HIGH_TOWER_LIFT_SPEED = tunable(0.5)

    LOW_TOWER_DESCEND_SPEED = tunable(-0.5)
    HIGH_TOWER_DESCEND_SPEED = tunable(-0.5)

    # required devices
    low_tower_motor: lazytalonsrx.LazyTalonSRX
    high_tower_motor: lazytalonsrx.LazyTalonSRX

    def __init__(self):
        self.desired_output = [0, 0]
        self.is_moving = False
        self.ball_count = [False, False, False, False]

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop()

    def lift(self) -> None:
        """Start lifting up balls."""
        self.is_moving = True
        self.desired_output = [self.LOW_TOWER_LIFT_SPEED, self.HIGH_TOWER_LIFT_SPEED]

    def descend(self) -> None:
        """Start bringing down balls."""
        self.is_moving = True
        self.desired_output = [
            self.LOW_TOWER_DESCEND_SPEED,
            self.HIGH_TOWER_DESCEND_SPEED,
        ]

    def stop(self) -> None:
        """Stop lifting balls."""
        self.is_moving = False
        self.desired_output = [0, 0]

    def hasBall(self, index: int) -> bool:
        """Does the tower have a ball at the given index."""
        return self.ball_count[index]

    def isFullyLoaded(self) -> bool:
        """Are 4 balls in the tower."""
        return all(self.ball_count)

    def execute(self):
        # TODO handle tower
        if self.is_moving:
            self.low_tower_motor.setOutput(self.desired_output[0])
            self.high_tower_motor.setOutput(self.desired_output[1])
        else:
            self.low_tower_motor.setOutput(0)
            self.high_tower_motor.setOutput(0)
        # TODO track_balls
        if False:  # trigger 0
            self.ball_count[0] = True
        if False:  # trigger 1
            self.ball_count[1] = True
        if False:  # trigger 2
            self.ball_count[2] = True
        if False:  # trigger 3
            self.ball_count[3] = True
