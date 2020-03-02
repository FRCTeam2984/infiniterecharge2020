from magicbot.state_machine import AutonomousStateMachine, state
from components import chassis, tower, intake
from statemachines import alignchassis, shooter, indexer


class Autonomous(AutonomousStateMachine):

    alignchassis: alignchassis.AlignChassis
    shooter: shooter.Shooter
    indexer: indexer.Indexer
    tower: tower.Tower
    intake: intake.Intake
    chassis: chassis.Chassis

    def __init__(self):
        super().__init__()
        pass

    def setup(self):
        pass

    def on_enable(self):
        super().on_enable()

    @state(first=True)
    def alignChassis(self, initial_call):
        if initial_call:
            pass
        self.alignchassis.engage()
        if self.alignchassis.isAligned():
            self.next_state("shootBalls")

    @state()
    def shootBalls(self, initial_call):
        if initial_call:
            pass
        self.shooter.engage()
        if self.tower.isEmpty():
            self.done()

    def done(self):
        super().done()
        self.chassis.stop()
        self.tower.stop(tower.TowerStage.BOTH)
        self.intake.stop()

