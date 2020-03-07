from networktables import NetworkTables

from components import intake, tower, turret
from components.tower import TowerStage
from magicbot.state_machine import StateMachine, state, timed_state
from utils import units


class SafeIntake(StateMachine):

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
            self.next_state("wait")

    @timed_state(duration=0.3, next_state="runIntake")
    def wait(self):
        pass

    def done(self):
        super().done()
        self.intake.stop()

    def execute(self):
        super().execute()


class Indexer(StateMachine):

    intake: intake.Intake
    tower: tower.Tower
    turret: turret.Turret

    TURRET_TOLERANCE = 5 * units.radians_per_degree

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
        self.tower.stop(TowerStage.BOTH)
        if self.tower.highTowerCount() == 0:
            # the high tower is empty
            # _____, 1____, _2___, __3__, 12___, _23__, 1_3__, 123__
            self.next_state("firstBall")
        elif self.tower.highTowerCount() == 1:
            # the high tower has 1 ball
            if self.tower.hasBalls([4]) and not self.tower.hasBalls([1, 2, 3]):
                # ___4_, 1__4_, _2_4_, __34_, 12_4_, _234_, 1_34_
                self.next_state("shiftUpHighTower")
            elif self.tower.hasBalls([5]) and not self.tower.hasBalls([1, 2, 3]):
                # ____5, 1___5, _2__5, __3_5, 12__5, _23_5, 1_3_5
                self.next_state("fillLowTower")
            elif self.tower.onlyHasBalls([1, 2, 3, 5]):
                # 123_5
                self.next_state("shiftDownHighTower")
            elif self.tower.onlyHasBalls([1, 2, 3, 4]):
                # 1234_
                self.next_state("shiftUpLowTower")
        elif self.tower.highTowerCount() == 2:
            # the high tower has 2 balls
            if self.tower.lowTowerCount != 3 and not self.tower.hasBalls([1]):
                # ___45, _2_45, __345, _2345
                self.next_state("shiftDownLowTower")
            elif (
                self.tower.lowTowerCount != 3
                and self.tower.hasBalls([1])
                and not self.tower.onlyHasBalls([1, 3, 4, 5])
                and self.intake.hasBall()
            ):
                # 01__45, # 012_45
                self.next_state("safelyFillLowTower")
            elif self.tower.onlyHasBalls([1, 3, 4, 5]):
                # 1_345
                # TODO give up???
                self.next_state("safelyFillLowTower")
            elif self.tower.isFull():
                # 12345
                self.next_state("towerFull")

    @state()
    def firstBall(self, initial_call):
        self.tower.intakeFast(TowerStage.BOTH)
        if self.tower.hasBalls([4]):
            self.next_state("handleBalls")

    @state()
    def shiftUpHighTower(self, initial_call):
        self.tower.intakeFast(TowerStage.HIGH)
        if self.tower.hasBalls([5]):
            self.next_state("handleBalls")

    @state()
    def shiftDownHighTower(self, initial_call):
        self.tower.unjam(TowerStage.HIGH)
        if self.tower.hasBalls([4]):
            self.next_state("handleBalls")

    @state()
    def fillLowTower(self, initial_call):
        self.tower.intakeFast(TowerStage.LOW)
        if self.tower.hasBalls([1, 2, 3]):
            self.next_state("handleBalls")

    @state()
    def safelyFillLowTower(self, initial_call):
        self.tower.intakeSlow(TowerStage.LOW)
        if self.tower.hasBalls([3]):
            self.next_state("handleBalls")

    @state()
    def shiftUpLowTower(self, initial_call):
        self.tower.intakeFast(TowerStage.BOTH)
        if self.tower.hasBalls([5]):
            self.next_state("handleBalls")

    @state()
    def shiftDownLowTower(self, initial_call):
        self.tower.unjam(TowerStage.LOW)
        if self.tower.hasBalls([1]):
            self.next_state("handleBalls")

    @state()
    def towerFull(self, initial_call):
        self.done()

    def done(self):
        super().done()
        self.tower.stop(TowerStage.BOTH)

    def execute(self):
        super().execute()
