import ctre


from utils import units


class LazyPigeonIMU(ctre.PigeonIMU):
    def __init__(self, master: ctre.WPI_TalonSRX):
        super().__init__(master)

    def _getRotation(self, axis: int) -> float:
        return self.getYawPitchRoll()[axis] * units.radians_per_degree

    def _getRotationInRange(self, axis: int) -> float:
        """Get the yaw in radians from -pi to pi."""
        rotation = self.getYawPitchRoll()[axis] * units.radians_per_degree
        return units.angle_range(rotation)

    def getHeading(self) -> float:
        """Get the heading in radians."""
        return self._getRotation(0)

    def getHeadingInRange(self) -> float:
        """Get the heading in radians from -pi to pi."""
        return self._getRotationInRange(0)

    def getPitch(self) -> float:
        """Get the pitch in radians."""
        return self._getRotation(1)

    def getPitchInRange(self) -> float:
        """Get the pitch in radians from -pi to pi."""
        return self._getRotationInRange(1)
