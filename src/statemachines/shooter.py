from magicbot.state_machine import StateMachine, state, timed_state
from controls import pidf
from components import vision, chassis, flywheel, tower, turret
from utils import lazypigeonimu, units, pose
import numpy as np
from wpilib import Timer


class Shooter(StateMachine):

    DISTANCES = (0, 3.5) # m
    RPMS = (0, 3500)

    chassis: chassis.Chassis
    tower: tower.Tower
    turret: turret.Turret
    flywheel: flywheel.Flywheel
    vision: vision.Vision

    def setup(self):
        self.balls_shot = 0
        self.chassis.setBreakMode()

    def isReadyToShoot(self):
        return self.turret.isReady() and self.flywheel.isReady()

    @state(first=True)
    def alignTurretAndSpinFlywheel(self, initial_call):
        if self.balls_shot == 4:
            self.next_state("success")
        if initial_call:
            self.turret.trackTarget()
            self.flywheel.setRPM(self.RPMS[1]) # np.interp(self.vision.getDistance(), self.DISTANCES, self.RPMS)
        if self.turret.isReady() and self.flywheel.isReady():
            self.next_state("feedBalls")

    @state()
    def feedBalls(self, initial_call):
        if self.balls_shot == 4:
            self.next_state("success")
        if initial_call:
            self.balls_shot += 1
            self.tower.index()
        if not self.isReadyToShoot():
            self.tower.stop()
            self.next_state("alignTurretAndSpinFlywheel")

    @state()
    def success(self):
        self.chassis.setCoastMode()
        self.tower.stop()
        self.turret.stop()
        self.flywheel.stop()
        self.done()
