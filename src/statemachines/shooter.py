from magicbot.state_machine import StateMachine, state, timed_state
from networktables import NetworkTables

from components import chassis, flywheel, tower, turret, vision
from utils import units


class TurretTracker(StateMachine):
    turret: turret.Turret
    vision: vision.Vision

    # search config
    SEARCH_MIN = -60 * units.radians_per_degree
    SEARCH_MAX = 60 * units.radians_per_degree
    SEARCH_SPEED = 0.3

    def __init__(self):
        self.is_searching_reverse = False

    def track(self):
        self.engage()

    @state(first=True)
    def searchForTarget(self, initial_call):
        """Slew the turret back and forth until a vision target is found."""
        if initial_call:
            # set initial search direction
            self.is_searching_reverse = self.turret.getHeading() <= 0

        # search forwards or reverse
        if self.is_searching_reverse:
            self.turret.setOutput(-self.SEARCH_SPEED)
        else:
            self.turret.setOutput(self.SEARCH_SPEED)

        # switch diections if min or max exceeded
        if self.turret.getHeading() <= self.SEARCH_MIN:
            self.is_searching_reverse = False
        elif self.turret.getHeading() >= self.SEARCH_MAX:
            self.is_searching_reverse = True

        # if a vision target is found, track it
        if self.vision.hasTarget():
            self.next_state("trackTarget")

    @state()
    def trackTarget(self, initial_call):
        """Move the turret to the vision target."""
        if self.vision.hasTarget():
            heading_error = self.vision.getHeading()
            self.turret.setRelativeHeading(-heading_error)
        else:
            self.next_state("searchForTarget")

    def execute(self):
        super().execute()
        if self.is_executing:
            self.vision.enableLED(True)

    def done(self):
        super().done()
        self.turret.stop()
        self.vision.enableLED(False)


class Shooter(StateMachine):

    chassis: chassis.Chassis
    tower: tower.Tower
    turret: turret.Turret
    flywheel: flywheel.Flywheel
    vision: vision.Vision

    VISION_TOLERANCE = 1.5 * units.radians_per_degree

    def on_disable(self):
        self.done()

    def setup(self):
        self.nt = NetworkTables.getTable("/components/shooter")

    def shoot(self):
        """Enable the statemachine."""
        self.engage()

    def isReadyToShoot(self):
        """Is the turret in position and flywheel up to speed."""
        return self.turret.isReady() and self.flywheel.isReady()

    @timed_state(first=True, duration=0.1, next_state="spinFlywheel")
    def unjamBalls(self, initial_call):
        if not self.tower.hasBalls([5]):
            self.next_state_now("spinFlywheel")
        self.tower.unjam(tower.TowerStage.BOTH)

    @state()
    def spinFlywheel(self, initial_call):
        """Spin the flywheel based on the distance to target."""
        self.tower.stop(tower.TowerStage.BOTH)
        if initial_call:
            distance = self.vision.getDistance()
            self.flywheel.setDistance(distance)
        if self.flywheel.isReady():
            self.next_state("feedBalls")

    @state()
    def feedBalls(self, initial_call):
        """Feed balls into the shooter."""
        if not self.flywheel.isReady():
            self.tower.stop(tower.TowerStage.BOTH)
            self.next_state("spinFlywheel")
        if (
            abs(self.vision.getHeading()) <= self.VISION_TOLERANCE
            and self.vision.hasTarget()
        ):
            self.tower.feed(tower.TowerStage.BOTH)
        else:
            self.tower.stop(tower.TowerStage.BOTH)

    def execute(self):
        super().execute()
        if self.is_executing:
            self.vision.enableLED(True)

    def done(self):
        super().done()
        self.tower.stop(tower.TowerStage.BOTH)
        self.turret.stop()
        self.flywheel.setRPM(0)
        self.vision.enableLED(False)
