import numpy as np


class PIDF:
    """A PIDF skeleton class."""

    def __init__(
        self,
        setpoint=0,
        kp=0,
        ki=0,
        kd=0,
        kf=0,
        continuous=False,
        min_in=-100,
        max_in=100,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kf = kf

        self.last_error = 0
        self.cur_error = 0

        self.integral = 0
        self.derivative = 0

        self.setpoint = setpoint
        self.output = 0
        self.continuous = continuous

        self.min_in = min_in
        self.max_in = max_in

        self.min_out = np.NINF
        self.max_out = np.Inf

    def update(self, input, dt):
        """Update the PIDF controller."""
        if dt < 1e-6:
            dt = 1e-6
        self.cur_error = self.setpoint - input

        if self.continuous and abs(self.cur_error) > (self.max_in - self.min_in) / 2:
            if self.cur_error > 0:
                self.cur_error = self.cur_error - self.max_in + self.min_in
            else:
                self.cur_error = self.cur_error + self.max_in - self.min_in

        self.proportion = self.cur_error
        self.integral = self.integral + (self.cur_error * dt)
        self.derivative = (self.cur_error - self.last_error) / dt

        output = (
            (self.kp * self.proportion)
            + (self.ki * self.integral)
            + (self.kd * self.derivative)
            + (self.kf * self.setpoint)
        )
        self.output = np.clip(output,self.min_out, self.max_out)
        return self.output

    def setSetpoint(self, setpoint: float):
        self.reset()
        self.setpoint = setpoint

    def setOutputRange(self, min_out: float, max_out: float):
        self.min_out = min_out
        self.max_out = max_out

    def reset(self):
        """Reset control values."""
        self.last_error = 0
        self.cur_error = 0

        self.integral = 0
        self.derivative = 0

        self.setpoint = 0
        self.output = 0
