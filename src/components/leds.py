import numpy as np
import wpilib

from components import chassis, flywheel, turret, vision


class LED:

    LENGTH_CHASSIS = 20
    LENGTH_TURRET = 20
    LENGTH_FLYWHEEL = 20
    LED_LENGTH = LENGTH_TURRET + LENGTH_CHASSIS + LENGTH_FLYWHEEL

    COLOR_OFF = wpilib._wpilib.AddressableLED.LEDData(0, 0, 0)
    COLOR_RED = wpilib._wpilib.AddressableLED.LEDData(255, 0, 0)
    COLOR_GREEN = wpilib._wpilib.AddressableLED.LEDData(0, 255, 0)
    COLOR_BLUE = wpilib._wpilib.AddressableLED.LEDData(0, 0, 255)

    UPDATE_INTERVAL = 0.2
    STRIPE_LENGTH_CHASSIS = 5
    STRIPE_LENGTH_TURRET = 5
    STRIPE_LENGTH_FLYWHEEL = 5

    # required devices
    led: wpilib.AddressableLED
    turret: turret.Turret
    flywheel: flywheel.Flywheel
    vision: vision.Vision
    chassis: chassis.Chassis

    def __init__(self):
        self.time = 0
        self.new_time = 0
        self.blink = False
        self.stripe_index = [0, 0, 0]
        self.lengths = (self.LENGTH_CHASSIS, self.LENGTH_TURRET, self.LENGTH_FLYWHEEL)

    def setup(self):
        self.chassis_data = [self.COLOR_OFF] * self.LENGTH_CHASSIS
        self.turret_data = [self.COLOR_OFF] * self.LENGTH_TURRET
        self.flywheel_data = [self.COLOR_OFF] * self.LENGTH_FLYWHEEL
        self.led_data = self.chassis_data + self.turret_data + self.flywheel_data
        self.led.setLength(self.LED_LENGTH)

    def on_enable(self):
        self.led.start()

    def test(self):
        pass

    def _getStripeData(self, rgb, index, stripe_length, length):
        data = [rgb] * stripe_length + [self.COLOR_OFF] * (length - stripe_length)
        return np.roll(data, index).tolist()

    def _getData(self, rgb, length):
        return [rgb] * length

    def execute(self):
        pass
        # self.time = wpilib.Timer.getFPGATimestamp()
        # if (self.time - self.new_time) >= self.UPDATE_INTERVAL:
        #     for i in range(0, len(self.stripe_index)):
        #         self.stripe_index[i] += 1
        #         if self.stripe_index[i] >= self.lengths[i]:
        #             self.stripe_index[i] = 0
        #     self.new_time = self.time
        #     self.blink = not self.blink

        # if self.turret.isSearching():
        #     self.turret_data = self._getStripeData(
        #         self.COLOR_BLUE,
        #         self.stripe_index[1],
        #         self.STRIPE_LENGTH_TURRET,
        #         self.LENGTH_TURRET,
        #     )
        # elif self.turret.isTrackingTarget():
        #     self.turret_data = self._getData(
        #         self.COLOR_BLUE if self.blink else self.COLOR_OFF, self.LENGTH_TURRET
        #     )
        # elif self.turret.isReady():
        #     self.turret_data = self._getData(self.COLOR_BLUE, self.LENGTH_TURRET)
        # else:
        #     self.turret_data = self._getData(self.COLOR_OFF, self.LENGTH_TURRET)

        # if self.flywheel.is_spinning:
        #     if self.flywheel.isReady():
        #         self.flywheel_data = self._getData(self.COLOR_RED, self.LENGTH_FLYWHEEL)
        #     else:
        #         self.flywheel_data = self._getData(
        #             self.COLOR_RED if self.blink else self.COLOR_OFF,
        #             self.LENGTH_FLYWHEEL,
        #         )
        # else:
        #     self.flywheel_data = self._getData(self.COLOR_OFF, self.LENGTH_FLYWHEEL)

        # if self.chassis.isAligning():
        #     if self.vision.isChassisReady():
        #         self.chassis_data = self._getData(self.COLOR_GREEN, self.LENGTH_CHASSIS)
        #     else:
        #         self.chassis_data = self._getData(
        #             self.COLOR_GREEN if self.blink else self.COLOR_OFF,
        #             self.LENGTH_CHASSIS,
        #         )
        # else:
        #     self.chassis_data = self._getData(self.COLOR_OFF, self.LENGTH_CHASSIS)

        # self.all_data = self.chassis_data + self.turret_data + self.flywheel_data
        # self.led.setData(self.all_data)
