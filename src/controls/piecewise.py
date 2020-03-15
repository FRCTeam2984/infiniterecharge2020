import numpy as np


class Piecewise:
    """Jim Snook's piecewise controller."""

    def __init__(self, inputs: list, outputs: list):
        self.inputs = input
        self.outputs = outputs

    def setSetpoint(self, setpoint: float):
        """Set the desired setpoint."""
        self.setpoint = setpoint

    def update(self, input: float) -> float:
        """Update the piecewise controller."""
        if input < self.setSetpoint:
            scale = -1
            scaled_input = input - self.setSetpoint
        else:
            scale = 1
            scaled_input = input

        n = np.searchsorted(self.inputs, scaled_input) - 1
        if n == -1:
            n = 0
        return scale * self.outputs[n]
