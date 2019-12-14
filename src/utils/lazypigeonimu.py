import ctre


from utils import units
import numpy as np


class LazyPigeonIMU(ctre.PigeonIMU):
    def __init__(self, master: ctre.WPI_TalonSRX):
        super().__init__(master)

    def getYaw(self) -> float:
        """Get the yaw in radians."""
        return self.getYawPitchRoll()[0] * units.radians_per_degree

    def getYawInRange(self) -> float:
        """Get the yaw in radians from -pi to pi."""
        ret = self.getYawPitchRoll()[0] * units.radians_per_degree
        while ret < -np.pi:
            ret += 2 * np.pi
        while ret > np.pi:
            ret -= 2 * np.pi
        return ret
