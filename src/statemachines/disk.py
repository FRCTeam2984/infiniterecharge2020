from networktables import NetworkTables

from components.spinner import Colors, Spinner
from magicbot.state_machine import StateMachine


class Disk(StateMachine):

    COLORS_PER_ROTATION = 8
    COLORS_FOR_ROTATION = COLORS_PER_ROTATION * 4

    spinner: Spinner

    def __init__(self):
        pass

    def on_disable(self):
        self.done()

    def setup(self):
        self.color = Colors.UNKOWN
        self.last_color = Colors.UNKOWN
        self.desired_color = Colors.UNKOWN
        self.color_count = 0
        self.nt = NetworkTables.getTable("/components/disk")

    def rotationControl(self):
        """Start stage two (spin 3-5 times)."""
        self.engage(initial_state="spinThreeToFive")

    def positionControl(self):
        """Start stage three (spin to color)."""
        self.engage(initial_state="spinToColor")

    def spinThreeToFive(self, initial_call):
        if initial_call:
            self.last_color = self.spinner.getColor()
            self.color_count = 0

        self.spinner.forward()

        self.color = self.spinner.getColor()

        if self.color != self.last_color:
            self.color_count += 1
        if self.color_count >= self.COLORS_FOR_ROTATION:
            self.done()

        self.last_color = self.color

    def spinToColor(self, initial_call):
        if initial_call:
            # TODO get desired color
            self.desired_color = Colors.UNKOWN

        self.spinner.forward()

        self.color = self.spinner.getColor()

        if self.desired_color == self.color:
            self.done()

    def done(self):
        super().done()
        self.spinner.stop()

    def execute(self):
        super().execute()
