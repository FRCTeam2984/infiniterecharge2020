from magicbot.state_machine import StateMachine, state, timed_state
from networktables import NetworkTables

from components import chassis, intake, tower, flywheel
from tower import TowerStage
from magicbot import tunable
import logging


class IntakeStateMachine(StateMachine):

    intake: intake.Intake

    def __init__(self):
        pass

    def on_disable(self):
        self.done()

    def setup(self):
        pass

    def intakeBalls(self):
        self.engage()

    @state(first=True)
    def runIntake(self, initial_call):
        self.intake.intake()
        if self.intake.hasBall():
            self.next_state("stopIntake")

    @state()
    def stopIntake(self, initial_call):
        self.intake.stop()
        if not self.intake.hasBall():
            self.next_state("stopIntake")

    def done(self):
        super().done()
        self.intake.stop()

    def execute(self):
        super().execute()


class Indexer(StateMachine):

    tower: tower.Tower

    def __init__(self):
        pass

    def on_disable(self):
        self.done()

    def setup(self):
        self.nt = NetworkTables.getTable("/components/indexer")

    def index(self):
        self.engage()

    @state(first=True)
    def handleBalls(self, initial_call):
        if self.tower.highTowerCount() == 0:
            # the high tower is empty
            if not self.tower.onlyHasBalls([3]):
                # _____, 1____, _2___, 12___
                # waiting for first ball to get into pos 3
                self.next_state("firstBall")
            elif self.tower.onlyHasBalls([3]) or self.tower.onlyHasBalls([1, 3]):
                # __3__, 1_3__
                # waiting for second ball to get into pos 2
                self.next_state("secondBall")
            elif self.tower.onlyHasBalls([2, 3]) or self.tower.onlyHasBalls([1, 2, 3]):
                # _23__, 123__
                # shift 2 balls from low to high tower
                self.next_state("shiftToHighTower")
        elif self.tower.highTowerCount() == 1:
            # the high tower has 1 ball
            if self.tower.hasBalls([5]):
                # ____5, 1___5, _2__5, __3_5, 12__5, _23_5, 1_3_5, 123_5
                # shift the ball in the high tower down
                self.next_state("shiftDownHighTower")
            elif self.tower.hasBalls([3, 4]):
                # __34_, 1_34_, _234_, 1234_
                # shift a ball from the low to high tower
                self.next_state("shiftToHighTower")
            elif not self.tower.hasBalls([3]) and self.tower.hasBalls(4):
                # ___4_, 1__4_, _2_4_, 12_4_
                self.next_state("firstBall")
        elif self.tower.highTowerCount() == 2:
            # the high tower is full
            if self.tower.lowTowerCount != 3:
                # ___45, 1__45, _2_45, __345, 12_45, _2345, 1_345
                # slowly load balls into the low tower to prevent jamming
                self.next_state("fillLowTower")
            elif self.tower.isFull():
                # 12345
                # the tower is full, the robot has 5 balls
                self.next_state("towerFull")

    @state()
    def firstBall(self, initial_call):
        self.tower.intakeFast(TowerStage.LOW)
        if self.tower.hasBall(3):
            self.next_state("handleBalls")

    @state()
    def secondBall(self, initial_call):
        self.tower.intakeFast(TowerStage.LOW)
        if self.tower.hasBall(2):
            self.next_state("handleBalls")

    @state()
    def shiftToHighTower(self, initial_call):
        self.tower.intakeFast(TowerStage.BOTH)
        if self.tower.hasBall(4) and self.tower.hasBall(5):
            self.next_state("handleBalls")

    @state()
    def fillLowTower(self, initial_call):
        self.tower.intakeSlow(TowerStage.LOW)
        if self.tower.hasBall(2) and self.tower.hasBall(3):
            self.next_state("handleBalls")

    @state()
    def towerFull(self, initial_call):
        self.done()

    def done(self):
        super().done()
        self.tower.stop(TowerStage.BOTH)

    def execute(self):
        super().execute()
