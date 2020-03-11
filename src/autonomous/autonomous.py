from magicbot.state_machine import AutonomousStateMachine, state, timed_state

from components import chassis, flywheel, intake, tower
from statemachines import alignchassis, indexer, shooter


class Autonomous(AutonomousStateMachine):

    MODE_NAME = "Shoot"
    DEFAULT = True

    alignchassis: alignchassis.AlignChassis
    shooter: shooter.Shooter
    turrettracker: shooter.TurretTracker
    indexer: indexer.Indexer
    tower: tower.Tower
    intake: intake.Intake
    chassis: chassis.Chassis
    flywheel: flywheel.Flywheel

    BACKUP_SPEED = -0.3
    BACKUP_DURATION = 1

    def __init__(self):
        super().__init__()

    def setup(self):
        pass

    def on_enable(self):
        super().on_enable()

    @timed_state(first=True, duration=BACKUP_DURATION, next_state="shootBalls")
    def moveAndAlign(self):
        self.chassis.setOutput(-self.BACKUP_SPEED, -self.BACKUP_SPEED)
        self.flywheel.revUp()
        self.turrettracker.engage()

    @state()
    def shootBalls(self, initial_call):
        if initial_call:
            self.chassis.stop()
        self.turrettracker.engage()
        self.shooter.engage()

    def done(self):
        super().done()
        self.chassis.stop()
        self.tower.stop(tower.TowerStage.BOTH)
        self.intake.stop()
