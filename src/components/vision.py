import logging
from enum import Enum
from networktables import NetworkTables
from utils import units, pose
import numpy as np
from magicbot import feedback


class Vision:

    # field and robot measurements
    TARGET_HEIGHT = 90 * units.meters_per_inch
    CAMERA_HEIGHT = 30 * units.meters_per_inch
    CAMERA_PITCH = 0 * units.radians_per_degree

    # desired setpoints and tolerances
    DISTANCE_DESIRED = 192 * units.meters_per_inch
    DISTANCE_TOLERANCE_FINE = 2 * units.meters_per_inch
    DISTANCE_TOLERANCE_COARSE = 12 * units.meters_per_inch

    HEADING_DESIRED = 0 * units.radians_per_degree
    HEADING_TOLERANCE_FINE = 1 * units.radians_per_degree
    HEADING_TOLERANCE_COARSE = 30 * units.radians_per_degree

    def __init__(self):
        self.limelight = NetworkTables.getTable("limelight")

    def on_enable(self):
        pass

    def getHeading(self) -> float:
        """Get the yaw offset to the target."""
        heading = (
            self.limelight.getNumber("tx", float("nan")) * units.radians_per_degree
        )
        return heading

    def getPitch(self) -> float:
        """Get the pitch offset to the target."""
        pitch_offset = (
            self.limelight.getNumber("ty", float("nan")) * units.radians_per_degree
        )
        return self.CAMERA_PITCH + pitch_offset

    def getDistance(self) -> float:
        """Get the distance offset to the target."""
        pitch = self.getPitch()
        dist = (self.TARGET_HEIGHT - self.CAMERA_HEIGHT) / np.tan(pitch)
        return dist

    def getHeadingError(self) -> float:
        """Get the error in the heading from the desired heading."""
        return self.getHeadingError() - self.HEADING_DESIRED

    def getDistanceError(self) -> float:
        """Get the error in the distance from the desired distance."""
        return self.getDistance() - self.DISTANCE_DESIRED

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

    def isHeadingInCoarseRange(self) -> bool:
        """Is the heading roughly where is should be."""

        return (
            abs(self.getHeading() - self.HEADING_DESIRED)
            <= self.HEADING_TOLERANCE_COARSE
        )

    def isHeadingInFineRange(self) -> bool:
        """Is the heading exactly where is should be."""
        return (
            abs(self.getHeading() - self.HEADING_DESIRED) <= self.HEADING_TOLERANCE_FINE
        )

    def isChassisReady(self) -> bool:
        """Is the chassis ready to shoot."""
        return self.isHeadingInCoarseRange() and self.isDistanceInCoarseRange()

    def isTurretReady(self) -> bool:
        """Is the turret ready to shoot."""
        return self.isHeadingInFineRange()

    def isReady(self) -> bool:
        """Is the entire robot ready to shoot balls."""
        return self.isChassisReady() and self.isTurretReady()

    @feedback
    def get_pitch(self):
        return self.getPitch() * units.degrees_per_radian

    @feedback
    def get_heading(self):
        return self.getHeading() * units.degrees_per_radian

    @feedback
    def get_distance(self):
        return self.getDistance() * units.inches_per_meter

    @feedback
    def is_chassis_ready(self):
        return self.isChassisReady()

    @feedback
    def is_turret_ready(self):
        return self.isTurretReady()

    def execute(self):
        pass
