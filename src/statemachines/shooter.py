from magicbot.state_machine import StateMachine, state, timed_state
from networktables import NetworkTables

from components import chassis, flywheel, tower, turret, vision
from utils import units


class Shooter(StateMachine):

    # search config
    SEARCH_MIN = -45 * units.radians_per_degree
    SEARCH_MAX = 45 * units.radians_per_degree
    SEARCH_SPEED = 0.2

    chassis: chassis.Chassis
    tower: tower.Tower
    turret: turret.Turret
    flywheel: flywheel.Flywheel
    vision: vision.Vision

    def __init__(self):
        self.is_searching_reverse = False

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

    @state(first="True")
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
            self.next_state("trackTargetAndSpinFlywheel")

    @state
    def trackTarget(self, initial_call):
        """Move the turret to the vision target."""
        if self.vision.hasTarget():
            heading_error = self.vision.getHeading()
            self.turret.setRelativeHeading(-heading_error)
            if self.turret.isReady():
                self.next_state("spinFlywheel")
        else:
            self.next_state("searchForTarget")

    @state
    def spinFlywheel(self, initial_call):
        """Spin the flywheel based on the distance to target."""
        # if initial_call:
        distance = self.vision.getDistance()
        self.flywheel.setDistance(distance)
        # if self.flywheel.isReady():
        # self.next_state("feedBalls")

    @timed_state(duration=5)
    def feedBalls(self, initial_call):
        """Feed balls into the shooter."""
        if not self.flywheel.isReady():
            self.tower.stop()
            self.next_state("spinFlywheel")
        else:
            self.tower.lift()

    def execute(self):
        super().execute()
        self.vision.enableLED(True)

    def done(self):
        super().done()
        self.tower.stop()
        self.turret.stop()
        self.flywheel.stop()
        self.vision.enableLED(False)

