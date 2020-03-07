from magicbot.state_machine import AutonomousStateMachine, state, timed_state

from components import chassis, intake, tower
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

    def __init__(self):
        super().__init__()

    def setup(self):
        pass

    def on_enable(self):
        super().on_enable()

    @timed_state(first=True, duration=0.2, next_state="shootBalls")
    def move(self):
        self.chassis.setOutput(-0.1,-0.1)

    # @state(first=True)
    # def alignChassis(self, initial_call):
    #     if initial_call:
    #         pass
    #     self.alignchassis.engage()
    #     if self.alignchassis.isAligned():
    #         self.next_state("shootBalls")

    @state()
    def shootBalls(self, initial_call):
        self.turrettracker.engage()
        self.shooter.engage()
        # if self.tower.isEmpty():
        #     self.done()

    def done(self):
        super().done()
        self.chassis.stop()
        self.tower.stop(tower.TowerStage.BOTH)
        self.intake.stop()
