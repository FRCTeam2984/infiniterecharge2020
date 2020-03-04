from magicbot.state_machine import StateMachine, state, timed_state
from networktables import NetworkTables

from components import slider, winch


class Climb(StateMachine):

    slider: slider.Slider
    winch: winch.Winch

    def __init__(self):
        pass

    def on_disable(self):
        self.done()

    def setup(self):
        self.nt = NetworkTables.getTable("/components/climb")

    def startClimb(self):
        """Start the climb."""
        self.engage(initial_state="extendHook")

    def endClimb(self):
        """End the climb."""
        self.engage(initial_state="retractSlider")

    @state(first=True)
    def extendHook(self, initial_call):
        """Extend up the slider."""
        self.slider.extend()
        self.winch.unwind()

    @timed_state(duration=2, next_state="windWinch")
    def retractSlider(self):
        self.slider.retract()

    @state()
    def windWinch(self):
        self.winch.wind()

    def done(self):
        super().done()
        self.slider.stop()
        self.winch.stop()

    def execute(self):
        super().execute()
