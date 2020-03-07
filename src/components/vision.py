import numpy as np
from magicbot import tunable
from networktables import NetworkTables

from utils import rollingaverage, units


class Vision:

    # field and robot measurements
    TARGET_HEIGHT = 90 * units.meters_per_inch
    CAMERA_HEIGHT = 36.75 * units.meters_per_inch
    CAMERA_PITCH = (
        -1.26055 * units.radians_per_degree
    )  # TODO tune: ty - atan((TARGET_HEIGHT - CAMERA_HEIGHT) / distance)
    CAMERA_HEADING = -2 * units.radians_per_degree  # TODO tune

    # rolling average config
    ROLLING_WINDOW = 5

    def __init__(self):
        self.limelight = NetworkTables.getTable("limelight")
        self.is_led_enabled = False
        self.heading_average = rollingaverage.RollingAverage(self.ROLLING_WINDOW)
        self.pitch_average = rollingaverage.RollingAverage(self.ROLLING_WINDOW)
        self.heading = 0
        self.pitch = 0

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
        return self.limelight.getNumber("tv", 0) == 1 and self.getArea() >= 2

    def getHeading(self) -> float:
        """Get the yaw offset to the target."""
        return self.heading

    def getPitch(self) -> float:
        """Get the pitch offset to the target."""
        return self.pitch

    def getDistance(self) -> float:
        """Get the distance offset to the target."""
        distance = (self.TARGET_HEIGHT - self.CAMERA_HEIGHT) / np.tan(self.pitch)
        return distance

    def getArea(self) -> float:
        """Get the distance offset to the target."""
        return self.limelight.getValue("ta",0)
        
    def updateNetworkTables(self):
        """Update network table values related to component."""
        self.nt.putNumber("heading", self.heading * units.degrees_per_radian)
        self.nt.putNumber("distance", self.getDistance() * units.inches_per_meter)
        self.nt.putNumber(
            "distance_in_bananas", self.getDistance() * units.bananas_per_meter
        )
        self.nt.putBoolean("has_target", self.hasTarget())
        self.nt.putNumber("area", self.getArea())

    def execute(self):
        self.is_led_enabled = self.limelight.getNumber("ledMode", 0) == 3

        heading = (
            self.limelight.getNumber("tx", 0) * units.radians_per_degree
            + self.CAMERA_HEADING
        )
        pitch = (
            self.limelight.getNumber("ty", 0) * units.radians_per_degree
            + self.CAMERA_PITCH
        )

        self.heading = heading  # self.heading_average.calculate(heading)
        self.pitch = pitch  # self.pitch_average.calculate(pitch)

        self.updateNetworkTables()
