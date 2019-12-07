import ctre


from utils import units


class LazyPigeonIMU(ctre.PigeonIMU):
    def __init__(self, master: ctre.WPI_TalonSRX):
        super().__init__(master)

    def getYaw(self) -> float:
        """Get the yaw in radians."""
        return self.getYawPitchRoll()[0] * units.radians_per_degree
