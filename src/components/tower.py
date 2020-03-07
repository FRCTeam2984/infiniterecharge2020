
import numpy as np
from networktables import NetworkTables

from magicbot import tunable
from utils import lazytalonsrx


class TowerStage:
    LOW = 0
    HIGH = 1
    BOTH = 2


class Tower:

    # speeds at which to run motors
    LT_FEED_SPEED = tunable(0.2)
    HT_FEED_SPEED = tunable(0.2)

    LT_INTAKE_SLOW_SPEED = tunable(0.2)
    HT_INTAKE_SLOW_SPEED = tunable(0.3)

    LT_INTAKE_FAST_SPEED = tunable(0.4)
    HT_INTAKE_FAST_SPEED = tunable(0.5)

    LT_UNJAM_SPEED = tunable(-0.5)
    HT_UNJAM_SPEED = tunable(-0.5)

    # required devices
    low_tower_motor: lazytalonsrx.LazyTalonSRX
    high_tower_motor: lazytalonsrx.LazyTalonSRX
    tower_sensors: list

    def __init__(self):
        self.desired_output_low = 0
        self.desired_output_high = 0

        self.is_moving = False
        self.ball_count = np.array([False, False, False, False, False])

    def setup(self):
        self.nt = NetworkTables.getTable(f"/components/tower")
        self.high_tower_motor.setInverted(False)
        self.low_tower_motor.setInverted(True)

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop(TowerStage.BOTH)

    def _setOutput(self, stage, output_low, output_high):
        if stage == TowerStage.LOW:
            self.desired_output_low = output_low
        elif stage == TowerStage.HIGH:
            self.desired_output_high = output_high
        elif stage == TowerStage.BOTH:
            self.desired_output_low = output_low
            self.desired_output_high = output_high

    def intakeSlow(self, stage) -> None:
        """Intake balls into the tower."""
        self._setOutput(stage, self.LT_INTAKE_SLOW_SPEED, self.HT_INTAKE_SLOW_SPEED)

    def intakeFast(self, stage) -> None:
        """Intake balls into the tower."""
        self._setOutput(stage, self.LT_INTAKE_FAST_SPEED, self.HT_INTAKE_FAST_SPEED)

    def unjam(self, stage) -> None:
        """Intake balls into the tower."""
        self._setOutput(stage, self.LT_UNJAM_SPEED, self.HT_UNJAM_SPEED)

    def feed(self, stage) -> None:
        """Intake balls into the tower."""
        self._setOutput(stage, self.LT_FEED_SPEED, self.HT_FEED_SPEED)

    def stop(self, stage) -> None:
        """Stop moving balls."""
        self._setOutput(stage, 0, 0)

    def hasBalls(self, indexes: list) -> bool:
        """Does the tower have a ball at the given index."""
        indexes = np.array(indexes) - 1
        return np.all(self.ball_count[indexes])

    def onlyHasBalls(self, indexes: int) -> bool:
        """Does the tower have a ball at the given index."""
        indexes = np.array(indexes) - 1
        unwanted_elements = np.delete(self.ball_count, indexes)
        return np.all(self.ball_count[indexes]) and not np.any(unwanted_elements)

    def isFull(self) -> bool:
        """Are 4 balls in the tower."""
        return np.all(self.ball_count)

    def isEmpty(self) -> bool:
        """Are no balls in the tower."""
        return not np.any(self.ball_count[1:])

    def lowTowerCount(self):
        return np.sum(self.ball_count[:3])

    def highTowerCount(self):
        return np.sum(self.ball_count[3:])

    def updateNetworkTables(self):
        """Update network table values related to component."""
        self.nt.putNumber("low_stator_current", self.low_tower_motor.getStatorCurrent())
        self.nt.putNumber(
            "high_stator_current", self.high_tower_motor.getStatorCurrent()
        )
        self.nt.putNumber("desired_low", self.desired_output_low)
        self.nt.putNumber("desired_high", self.desired_output_high)
        self.nt.putBoolean("is_full", self.isFull())
        self.nt.putBoolean("is_empty", self.isEmpty())
        self.nt.putNumber("low_tower_count", self.lowTowerCount())
        self.nt.putNumber("high_tower_count", self.highTowerCount())
        self.nt.putBoolean("ball_1", self.ball_count[0])
        self.nt.putBoolean("ball_2", self.ball_count[1])
        self.nt.putBoolean("ball_3", self.ball_count[2])
        self.nt.putBoolean("ball_4", self.ball_count[3])
        self.nt.putBoolean("ball_5", self.ball_count[4])

    def execute(self):
        self.low_tower_motor.setOutput(self.desired_output_low)
        self.high_tower_motor.setOutput(self.desired_output_high)

        for i in range(0, len(self.tower_sensors)):
            self.ball_count[i] = not self.tower_sensors[i].get()
        self.updateNetworkTables()
