from magicbot.state_machine import StateMachine, state, timed_state
from controls import pidf
from components import vision, chassis, flywheel, tower, turret
from utils import lazypigeonimu, units
import numpy as np
from wpilib import Timer
from networktables import NetworkTables


class Shooter(StateMachine):

    DISTANCES = (
        np.array((6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16)) * units.meters_per_inch
    )
    RPMS = np.array((2475, 2475, 2350, 2250, 2275, 2300, 2260, 2300, 2320, 2330, 2335))

    chassis: chassis.Chassis
    tower: tower.Tower
    turret: turret.Turret
    flywheel: flywheel.Flywheel
    vision: vision.Vision

    def setup(self):
        self.chassis.setBreakMode()
        self.balls_shot = 0
        self.desired_rpm = 0
        self.nt = NetworkTables.getTable("/components/shooter")

    def shoot(self):
        self.engage()

    def isReadyToShoot(self):
        return self.turret.isReady() and self.flywheel.isReady()

    @state(first=True)
    def alignTurretAndSpinFlywheel(self, initial_call):
        if self.balls_shot == 4:
            self.done()
        if initial_call:
            self.turret.trackTarget()
        self.desired_rpm = np.interp(
            self.vision.getDistance(), self.DISTANCES, self.RPMS
        )
        self.flywheel.setRPM(self.desired_rpm)
        if self.turret.isReady() and self.flywheel.isReady():
            self.next_state("feedBalls")

    @state()
    def feedBalls(self, initial_call):
        if self.balls_shot == 4:
            self.done()
        if initial_call:
            self.balls_shot += 1
            self.tower.lift()
        if not self.isReadyToShoot():
            self.tower.stop()
            self.next_state("alignTurretAndSpinFlywheel")

    @state()
    def done(self):
        super.done()
        self.chassis.setCoastMode()
        self.tower.stop()
        self.turret.stop()
        self.flywheel.stop()
        self.done()
