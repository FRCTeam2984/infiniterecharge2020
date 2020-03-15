import numpy as np
from magicbot import tunable
from networktables import NetworkTables

from utils import rollingaverage, units


class Vision:

    # field and robot measurements
    TARGET_HEIGHT = 89.75 * units.meters_per_inch
    TARGET_WIDTH = 30 * units.meters_per_inch
    CAMERA_HEIGHT = 36.75 * units.meters_per_inch
    CAMERA_PITCH = (
        -1.5311254 * units.radians_per_degree
    )  # TODO tune: ty - atan((TARGET_HEIGHT - CAMERA_HEIGHT) / distance)
    CAMERA_HEADING = tunable(-2)  # * units.radians_per_degree  # TODO tune

    # rolling average config
    ROLLING_WINDOW = 1

    # force LED on
    FORCE_LED = True

    # fov
    FOV_X = 59.6 * units.radians_per_degree
    FOV_Y = 45.7 * units.radians_per_degree

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
        pass

    def on_disable(self):
        self.enableLED(False)

    def enableLED(self, value: bool) -> None:
        """Toggle the limelight LEDs on or off."""
        mode = 3 if (value or self.FORCE_LED) else 1
        self.limelight.putNumber("ledMode", mode)

    def isLEDEnabled(self) -> bool:
        return self.is_led_enabled

    def hasTarget(self) -> bool:
        """Has the limelight found a valid target."""
        return self.limelight.getNumber("tv", 0) == 1

    def getHeading(self) -> float:
        """Get the yaw offset to the target."""
        heading = (
            self.limelight.getNumber("tx", 0) * units.radians_per_degree
            + self.CAMERA_HEADING * units.radians_per_degree
        )
        self.heading = heading  # self.heading_average.calculate(heading)
        return self.heading

    def getPitch(self) -> float:
        """Get the pitch offset to the target."""
        pitch = (
            self.limelight.getNumber("ty", 0) * units.radians_per_degree
            + self.CAMERA_PITCH
        )
        self.pitch = pitch  # self.pitch_average.calculate(pitch)
        return self.pitch

    def getDistance(self) -> float:
        """Get the distance offset to the target."""
        if not self.hasTarget():
            return np.inf
        distance = (self.TARGET_HEIGHT - self.CAMERA_HEIGHT) / np.tan(self.getPitch())
        return distance

    def getDistanceFromArea(self) -> float:
        percent_x = self.limelight.getNumber("thor", 0) / 320
        radians_x = self.FOV_X * percent_x
        self.nt.putNumber("percent_x", percent_x)
        self.nt.putNumber("radians_x", radians_x * units.degrees_per_radian)
        if not self.hasTarget() or radians_x == 0:
            return np.inf
        radius = self.TARGET_WIDTH / radians_x
        return radius

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
        self.nt.putNumber("distance_feet", self.getDistance() * units.feet_per_meter)
        self.nt.putNumber("distance", self.getDistance() * units.inches_per_meter)

        self.nt.putNumber(
            "distance_from_area", self.getDistanceFromArea() * units.inches_per_meter
        )
        self.nt.putBoolean("has_target", self.hasTarget())
        self.nt.putNumber("area", self.getArea())
        self.nt.putNumber("ratio", self.getRatio())

    def execute(self):
        self.updateNetworkTables()
