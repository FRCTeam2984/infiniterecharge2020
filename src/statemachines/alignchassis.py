from magicbot.state_machine import StateMachine, state
from components import vision, chassis, flywheel, tower, turret
from utils import units, drivesignal, lazypigeonimu
from controls import pidf
import numpy as np
from networktables import NetworkTables

import wpilib


class AlignChassis(StateMachine):

    # target tracking pidf gains
    HEADING_KP = 1
    HEADING_KI = 0
    HEADING_KD = 0
    HEADING_KF = 0

    DISTANCE_KP = 4.5
    DISTANCE_KI = 0
    DISTANCE_KD = 0
    DISTANCE_KF = 0

    chassis: chassis.Chassis
    turret: turret.Turret
    vision: vision.Vision
    imu: lazypigeonimu.LazyPigeonIMU
    def __init__(self):
        self.desired_velocity = drivesignal.DriveSignal()
        self.prev_time = 0

    def setup(self):
        self.heading_pidf = pidf.PIDF(
            self.vision.HEADING_DESIRED,
            self.HEADING_KP,
            self.HEADING_KI,
            self.HEADING_KD,
            self.HEADING_KF,
            True,
            -np.pi,
            np.pi,
        )
        self.heading_pidf.setOutputRange(-0.4, 0.4)
        self.distance_pidf = pidf.PIDF(
            self.vision.DISTANCE_DESIRED,
            self.DISTANCE_KP,
            self.DISTANCE_KI,
            self.DISTANCE_KD,
            self.DISTANCE_KF,
        )
        self.distance_pidf.setOutputRange(-1, 1)


    def setup(self):
        self.nt = NetworkTables.getTable("/components/alignchassis")

    def align(self):
        self.engage()

    def isAligned(self):
        return self.vision.isChassisReady()

    def getAllocentricHeading(self):
        return (
            self.vision.getHeading()
            -self.turret.getHeading()
            - self.imu.getHeading()
        )

    @state(first=True)
    def driveToTarget(self, initial_call):
        if initial_call:
            self.prev_time = wpilib.Timer.getFPGATimestamp()
            self.chassis.setCoastMode()

        cur_time = wpilib.Timer.getFPGATimestamp()
        # dt = cur_time - self.prev_time
        dt = 0.02

        # calculate pidf outputs
        distance = self.vision.getDistance()
        # logging.info(distance_error)
        heading = self.vision.getHeading()

        distance_adjust = self.distance_pidf.update(distance, dt)
        heading_adjust = self.heading_pidf.update(heading, dt)

        # calculate wheel velocities and set motor outputs
        self.desired_velocity.left = distance_adjust + heading_adjust
        self.desired_velocity.right = distance_adjust - heading_adjust

        if self.isAligned():
            self.next_state("lockAtTarget")
        else:
            self.chassis.setVelocity(
                self.desired_velocity.left, self.desired_velocity.right
            )
            self.prev_time = cur_time

    @state()
    def lockAtTarget(self, initial_call):
        if initial_call:
            self.chassis.setBreakMode()
        if not self.isAligned():
            self.next_state("driveToTarget")
        else:
            self.chassis.stop()

    def done(self):
        super().done()
        self.chassis.setCoastMode()
        self.chassis.stop()
