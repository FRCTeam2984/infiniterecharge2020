import numpy as np
from networktables import NetworkTables

from utils import rollingaverage, units
from magicbot import tunable

class Vision:

    # field and robot measurements
    TARGET_HEIGHT = 90 * units.meters_per_inch # 54
    CAMERA_HEIGHT = 36.75 * units.meters_per_inch # 36.75 
    CAMERA_PITCH = (
        -3.2936549 * units.radians_per_degree
    )  # TODO tune: ty - atan((TARGET_HEIGHT - CAMERA_HEIGHT) / distance)
    CAMERA_HEADING = -2 * units.radians_per_degree  # TODO tune

    # rolling average config
    ROLLING_WINDOW = 1

    def __init__(self):
        self.limelight = NetworkTables.getTable("limelight")
        self.is_led_enabled = False
        self.heading = 0
        self.pitch = 0

    def setup(self):
        self.nt = NetworkTables.getTable(f"/components/vision")
        self.heading_average = rollingaverage.RollingAverage(self.ROLLING_WINDOW)
        self.pitch_average = rollingaverage.RollingAverage(self.ROLLING_WINDOW)

    def on_enable(self):
        self.heading_average = rollingaverage.RollingAverage(self.ROLLING_WINDOW)
        self.pitch_average = rollingaverage.RollingAverage(self.ROLLING_WINDOW)
        self.enableLED(False)

    def on_disable(self):
        self.enableLED(False)

    def enableLED(self, value: bool) -> None:
        """Toggle the limelight LEDs on or off."""
        mode = 3 if value else 1
        mode = 3
        self.limelight.putNumber("ledMode", mode)

    def isLEDEnabled(self) -> bool:
        return self.is_led_enabled

    def hasTarget(self) -> bool:
        """Has the limelight found a valid target."""
        return (
            self.limelight.getNumber("tv", 0) == 1
        )

    def getHeading(self) -> float:
        """Get the yaw offset to the target."""
        heading = (
            self.limelight.getNumber("tx", 0) * units.radians_per_degree
            + self.CAMERA_HEADING
        )
        self.heading = self.heading_average.calculate(heading)
        return self.heading

    def getPitch(self) -> float:
        """Get the pitch offset to the target."""
        pitch = (
            self.limelight.getNumber("ty", 0) * units.radians_per_degree
            + self.CAMERA_PITCH
        )
        self.pitch = self.pitch_average.calculate(pitch)
        return self.pitch

    def getDistance(self) -> float:
        """Get the distance offset to the target."""
        distance = (self.TARGET_HEIGHT - self.CAMERA_HEIGHT) / np.tan(self.getPitch())
        return distance

    def getArea(self) -> float:
        """Get the distance offset to the target."""
        return self.limelight.getValue("ta", 0)

    def getRatio(self) -> float:
        short = self.limelight.getValue("tshort", 0)
        short = np.inf if short == 0 else short
        return self.limelight.getValue("tlong", 0) / short

    def updateNetworkTables(self):
        """Update network table values related to component."""
        self.nt.putNumber("heading", self.heading * units.degrees_per_radian)
        self.nt.putNumber("pitch", self.pitch * units.degrees_per_radian)
        self.nt.putNumber("distance", self.getDistance() * units.inches_per_meter)
        self.nt.putNumber(
            "distance_in_bananas", self.getDistance() * units.bananas_per_meter
        )
        self.nt.putBoolean("has_target", self.hasTarget())
        self.nt.putNumber("area", self.getArea())
        self.nt.putNumber("ratio", self.getRatio())

    def execute(self):
        # self.is_led_enabled = self.limelight.getNumber("ledMode", 0) == 3

        self.updateNetworkTables()
