import numpy as np
from magicbot import tunable
from networktables import NetworkTables

from utils import units


class Vision:

    # field and robot measurements
    TARGET_HEIGHT = 90 * units.meters_per_inch
    CAMERA_HEIGHT = 36.75 * units.meters_per_inch
    CAMERA_PITCH = (
        -1.26055 * units.radians_per_degree
    )  # TODO tune: ty - atan((TARGET_HEIGHT - CAMERA_HEIGHT) / distance)
    CAMERA_HEADING = tunable(-2)  # TODO tune

    def __init__(self):
        self.limelight = NetworkTables.getTable("limelight")
        self.is_led_enabled = False

    def setup(self):
        self.nt = NetworkTables.getTable(f"/components/vision")

    def on_enable(self):
        self.enableLED(False)

    def on_disable(self):
        self.enableLED(False)

    def enableLED(self, value: bool) -> None:
        """Toggle the limelight LEDs on or off."""
        mode = 3 if value else 1
        self.limelight.putNumber("ledMode", mode)

    def isLEDEnabled(self) -> bool:
        return self.is_led_enabled

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

    def updateNetworkTables(self):
        """Update network table values related to component."""
        self.nt.putNumber("heading", self.getHeading() * units.degrees_per_radian)
        self.nt.putNumber("distance", self.getDistance() * units.inches_per_meter)
        self.nt.putNumber(
            "distance_in_bananas", self.getDistance() * units.bananas_per_meter
        )
        self.nt.putNumber("has_target", self.hasTarget())

    def execute(self):
        self.is_led_enabled = self.limelight.getNumber("ledMode", 0) == 3
        self.updateNetworkTables()
