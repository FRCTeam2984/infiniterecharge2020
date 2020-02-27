from networktables import NetworkTables

from components import intake, tower, chassis
from magicbot.state_machine import StateMachine, state, timed_state


class Indexer(StateMachine):

    UNJAM_DURATION = 0.5

    intake: intake.Intake
    tower:  tower.Tower
    chassis: chassis.Chassis

    def __init__(self):
        pass

    def on_disable(self):
        self.done()

    def setup(self):
        self.nt = NetworkTables.getTable("/components/indexer")

    def index(self):
        self.engage()

    def unjam(self):
        self.engage(initial_state="unjamTower")
        
    @state(first=True)
    def intakeBalls(self, initial_call):
        if initial_call:
            pass
        self.intake.intake()
        self.tower.intake()

    @timed_state(duration=UNJAM_DURATION)
    def unjamTower(self, initial_call):
        self.tower.unjam()

    def done(self):
        super().done()
        self.intake.stop()
        self.tower.stop()

    def execute(self):
        super().execute()
