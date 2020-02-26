from networktables import NetworkTables

from components import slider, winch
from magicbot.state_machine import StateMachine, timed_state


class Climb(StateMachine):

    slider: slider.Slider
    winch: winch.Winch

    def __init__(self):
        self.distance_adjust = 0
        self.heading_adjust = 0
        self.prev_time = 0
        self.desired_distance = 0

    def on_disable(self):
        self.done()

    def setup(self):
        self.nt = NetworkTables.getTable("/components/climb")

    def extendHook(self):
        """Start the climb."""
        self.engage(initial_state="moveSlider")

    def climb(self):
        """End the climb."""
        self.engage(initial_state="activateWinch")

    @timed_state(duration=5, first=True)
    def moveSlider(self, initial_call):
        """Extend up the slider."""
        self.slider.extend()

    @timed_state(duration=5)
    def activateWinch(self, initial_call):
        """Activate the winch to lift up the robot."""
        self.winch.winch()

    def done(self):
        super().done()
        self.slider.stop()
        self.winch.stop()

    def execute(self):
        super().execute()
