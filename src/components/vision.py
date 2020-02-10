import logging
from enum import Enum
from networktables import NetworkTables
from utils import units, pose
import numpy as np


class Vision:

    TARGET_HEIGHT = 90 * units.meters_per_inch
    CAMERA_HEIGHT = 30 * units.meters_per_inch
    CAMERA_PITCH = 0 * units.radians_per_degree

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
        heading = self.limelight.getNumber("tx",float('nan')) * units.radians_per_degree
        return heading

    def getPitch(self) -> float:
        pitch_offset = self.limelight.getNumber("ty",float('nan')) * units.radians_per_degree
        return self.CAMERA_PITCH + pitch_offset

    def getDistance(self) -> float:
        pitch = self.getPitch()
        dist = (self.TARGET_HEIGHT - self.CAMERA_HEIGHT) / np.tan(pitch)
        return dist

    def getHeadingError(self):
        return self.getHeadingError() - self.HEADING_DESIRED

    def getDistanceError(self):
        return self.getDistance() - self.DISTANCE_DESIRED

    def isDistanceInCoarseRange(self):
        return (
            abs(self.getDistance() - self.DISTANCE_DESIRED)
            <= self.DISTANCE_TOLERANCE_COARSE
        )

    def isDistanceInFineRange(self):
        return (
            abs(self.getDistance() - self.DISTANCE_DESIRED)
            <= self.DISTANCE_TOLERANCE_FINE
        )

    def isHeadingInCoarseRange(self):
        return (
            abs(self.getHeading() - self.HEADING_DESIRED)
            <= self.HEADING_TOLERANCE_COARSE
        )

    def isHeadingInFineRange(self):
        return (
            abs(self.getHeading() - self.HEADING_DESIRED) <= self.HEADING_TOLERANCE_FINE
        )

    def isChassisReady(self):
        return self.isHeadingInCoarseRange() and self.isDistanceInCoarseRange()

    def isTurretReady(self):
        return self.isHeadingInFineRange()

    def isReady(self):
        return self.isChassisReady() and self.isTurretReady()

    def execute(self):
        pass
