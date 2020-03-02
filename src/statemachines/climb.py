from magicbot.state_machine import StateMachine, state
from networktables import NetworkTables

from components import slider, winch
from utils import lazypigeonimu, units


class Climb(StateMachine):

    PITCH_OFFSET = 15 * units.radians_per_degree
    ROLL_OFFSET = 15 * units.radians_per_degree

    slider: slider.Slider
    winch: winch.Winch

    imu: lazypigeonimu.LazyPigeonIMU

    def __init__(self):
        self.distance_adjust = 0
        self.heading_adjust = 0
        self.prev_time = 0
        self.desired_distance = 0

    def on_disable(self):
        self.done()

    def setup(self):
        self.nt = NetworkTables.getTable("/components/climb")

    # def extendHook(self):
    #     """Start the climb."""
    #     self.engage(initial_state="extendSlider")

    # def climb(self):
    #     """End the climb."""
    #     self.engage(initial_state="windWinch")

    # def extendHook(self):
    #     """Start the climb."""
    #     self.engage(initial_state="extendSlider")

    # def climb(self):
    #     """End the climb."""
    #     self.engage(initial_state="windWinch")

    @state(first=True)
    def synchronousExtension(self, initial_call):
        """Extend up the slider."""
        self.slider.extend()
        self.winch.winch()

    def extendSlider(self, initial_call):
        """Extend up the slider."""
        self.slider.extend()

    @state()
    def retractSlider(self, initial_call):
        """Extend up the slider."""
        self.slider.retract()

    @state()
    def windWinch(self, initial_call):
        """Activate the winch to lift up the robot."""
        self.winch.wind()

    @state()
    def unwindWinch(self, initial_call):
        """Activate the winch to lift up the robot."""
        self.winch.unwind()

    def done(self):
        super().done()
        self.slider.stop()
        self.winch.stop()

    def execute(self):
        super().execute()
