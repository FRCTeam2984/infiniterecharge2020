import ctre

from utils import units


class LazyPigeonIMU(ctre.PigeonIMU):
    """A wrapper for the PigeonIMU."""

    def __init__(self, master: ctre.WPI_TalonSRX):
        super().__init__(master)

    def _getRotation(self, axis: int) -> float:
        """Get the angle of the given axis."""
        return self.getYawPitchRoll()[1][axis]

    def _getRotationInRange(self, axis: int) -> float:
        """Get the angle of the given axis within [-pi, pi]."""
        rotation = self._getRotation(axis)
        return units.angle_range(rotation)

    def getHeading(self) -> float:
        """Get the heading (yaw) in radians."""
        return self._getRotation(0)

    def getHeadingInRange(self) -> float:
        """Get the heading (yaw) in radians from -pi to pi."""
        return self._getRotationInRange(0)

    def getPitch(self) -> float:
        """Get the pitch in radians."""
        return self._getRotation(1)

    def getPitchInRange(self) -> float:
        """Get the pitch in radians from -pi to pi."""
        return self._getRotationInRange(1)

    def getRoll(self) -> float:
        """Get the roll in radians."""
        return self._getRotation(2)

    def getRollInRange(self) -> float:
        """Get the roll in radians from -pi to pi."""
        return self._getRotationInRange(2)
