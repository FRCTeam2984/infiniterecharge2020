from utils import lazytalonsrx
from magicbot import tunable
from networktables import NetworkTables


class Tower:

    # speeds at which to run motors
    FEED_SPEED = 0.2
    LOW_TOWER_FEED_SPEED = tunable(FEED_SPEED)
    HIGH_TOWER_FEED_SPEED = tunable(FEED_SPEED)

    INTAKE_SPEED = 0.5
    LOW_TOWER_INTAKE_SPEED = tunable(INTAKE_SPEED)
    HIGH_TOWER_INTAKE_SPEED = tunable(INTAKE_SPEED)

    UNJAM_SPEED = -0.5
    LOW_TOWER_UNJAM_SPEED = tunable(UNJAM_SPEED)
    HIGH_TOWER_UNJAM_SPEED = tunable(UNJAM_SPEED)

    # required devices
    low_tower_motor: lazytalonsrx.LazyTalonSRX
    high_tower_motor: lazytalonsrx.LazyTalonSRX

    def __init__(self):
        self.desired_output = [0, 0]
        self.is_moving = False
        self.ball_count = [False, False, False, False]

    def setup(self):
        self.nt = NetworkTables.getTable(f"/components/intake")
        self.high_tower_motor.setInverted(True)

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop()

    def intake(self) -> None:
        """Intake balls into the tower."""
        self.is_moving = True
        self.desired_output = [
            self.LOW_TOWER_INTAKE_SPEED,
            self.HIGH_TOWER_INTAKE_SPEED,
        ]

    def unjam(self) -> None:
        """Unjam balls by running tower backwards."""
        self.is_moving = True
        self.desired_output = [self.LOW_TOWER_UNJAM_SPEED, self.HIGH_TOWER_UNJAM_SPEED]

    def feed(self) -> None:
        """Feed balls into the shooter."""
        self.is_moving = True
        self.desired_output = [
            self.LOW_TOWER_FEED_SPEED,
            self.HIGH_TOWER_FEED_SPEED,
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

    def updateNetworkTables(self):
        """Update network table values related to component."""
        self.nt.putNumber("low_stator_current", self.low_tower_motor.getStatorCurrent())
        self.nt.putNumber(
            "high_stator_current", self.high_tower_motor.getStatorCurrent()
        )

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
        self.updateNetworkTables()
