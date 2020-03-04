import numpy as np


class RollingAverage:
    def __init__(self, window):
        self.window = window
        self.buffer = np.zeros(self.window)
        self.average = 0

    def calculate(self, value):
        self.buffer = np.roll(self.buffer, -1)
        self.buffer[-1] = value
        self.average = np.mean(self.buffer)
        return self.average