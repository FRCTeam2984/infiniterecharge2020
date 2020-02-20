import numpy as np
from networktables import NetworkTables

from utils import units


class Vision:

    # field and robot measurements
    TARGET_HEIGHT = 90 * units.meters_per_inch
    CAMERA_HEIGHT = 36.75 * units.meters_per_inch
    CAMERA_PITCH = (
        -1.65 * units.radians_per_degree
    )  # empirically calculated: ty - atan((TARGET_HEIGHT - CAMERA_HEIGHT) / distance)
    CAMERA_HEADING = -1
    # desired setpoints and tolerances
    DISTANCE_DESIRED = 144 * units.meters_per_inch
    DISTANCE_TOLERANCE_FINE = 2 * units.meters_per_inch
    DISTANCE_TOLERANCE_COARSE = 12 * units.meters_per_inch

    HEADING_DESIRED = 0 * units.radians_per_degree
    HEADING_TOLERANCE_FINE = 1 * units.radians_per_degree

    def __init__(self):
        self.limelight = NetworkTables.getTable("limelight")

    def setup(self):
        self.nt = NetworkTables.getTable(
            f"/components/{self.__class__.__name__.lower()}"
        )

    def on_enable(self):
        pass

    def enableLED(self, value: bool):
        """Toggle the limelight LEDs on or off."""
        mode = 3 if value else 1
        self.nt.putNumber("ledMode", mode)

    def hasTarget(self) -> bool:
        """Has the limelight found a valid target."""
        return self.limelight.getNumber("tv", 0) == 1

    def getHeading(self) -> float:
        """Get the yaw offset to the target."""
        heading = (
            self.limelight.getNumber("tx", np.nan) + self.CAMERA_HEADING
        ) * units.radians_per_degree
        return heading

    def getPitch(self) -> float:
        """Get the pitch offset to the target."""
        pitch_offset = self.limelight.getNumber("ty", np.nan) * units.radians_per_degree
        return self.CAMERA_PITCH + pitch_offset

    def getDistance(self) -> float:
        """Get the distance offset to the target."""
        pitch = self.getPitch()
        distance = (self.TARGET_HEIGHT - self.CAMERA_HEIGHT) / np.tan(pitch)
        return distance

    def isDistanceInCoarseRange(self) -> bool:
        """Is the distance roughly where is should be."""
        return (
            abs(self.getDistance() - self.DISTANCE_DESIRED)
            <= self.DISTANCE_TOLERANCE_COARSE
        )

    def isDistanceInFineRange(self) -> bool:
        """Is the distance exactly where is should be."""
        return (
            abs(self.getDistance() - self.DISTANCE_DESIRED)
            <= self.DISTANCE_TOLERANCE_FINE
        )

    def isHeadingInRange(self) -> bool:
        """Is the heading exactly where is should be."""
        return (
            abs(self.getHeading() - self.HEADING_DESIRED) <= self.HEADING_TOLERANCE_FINE
        )

    def isChassisReady(self) -> bool:
        """Is the chassis ready to shoot."""
        return self.isDistanceInCoarseRange()

    def isTurretReady(self) -> bool:
        """Is the turret ready to shoot."""
        return self.isHeadingInRange()

    def isReady(self) -> bool:
        """Is the entire robot ready to shoot balls."""
        return self.isChassisReady() and self.isTurretReady()

    def updateNetworkTables(self):
        self.nt.putValue("heading", self.getHeading() * units.degrees_per_radian)
        self.nt.putValue("distance", self.getDistance() * units.inches_per_meter)
        self.nt.putValue("distance_in_bananas", self.getDistance() * units.bananas_per_meter)
        self.nt.putValue("pitch", self.getPitch() * units.degrees_per_radian)
        self.nt.putValue("has_target", self.hasTarget())

    def execute(self):
        self.updateNetworkTables()
