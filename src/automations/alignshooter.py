from magicbot.state_machine import StateMachine, state, timed_state
from controls import pidf
from components import vision, chassis, shooter, tower, turret
from utils import lazypigeonimu, units, pose
import numpy as np
from wpilib import Timer


class AlignShooter(StateMachine):
    DESIRED_HEADING = 0 * units.radians_per_degree
    HEADING_KP = 0
    HEADING_KI = 0
    HEADING_KD = 0
    HEADING_KF = 0
    HEADING_TOLERANCE = 45 * units.radians_per_degree

    DESIRED_DISTANCE = 192 * units.meters_per_inch
    DISTANCE_KP = 0
    DISTANCE_KI = 0
    DISTANCE_KD = 0
    DISTANCE_KF = 0
    DISTANCE_TOLERANCE = 12 * units.meters_per_inch

    chassis: chassis.Chassis
    tower: tower.Tower
    turret: turret.Turret
    shooter: shooter.Shooter
    vision: vision.Vision

    gyro: lazypigeonimu.LazyPigeonIMU
    vision_offset: pose.Pose

    def setup(self):
        self.heading_pidf = pidf.PIDF(
            self.DESIRED_HEADING,
            self.HEADING_KP,
            self.HEADING_KI,
            self.HEADING_KD,
            self.HEADING_KF,
            True,
            -np.pi,
            np.pi,
        )

        self.distance_pidf = pidf.PIDF(
            self.DESIRED_DISTANCE,
            self.DISTANCE_KP,
            self.DISTANCE_KI,
            self.DISTANCE_KD,
            self.DISTANCE_KF,
        )

        self.cur_time = 0
        self.last_time = 0

    @state(first=True)
    def initializeAlignment(self):
        self.turret.setVisionTarget()

    @state()
    def steerTowardTarget(self, initial_call):
        if initial_call:
            self.last_time = Timer.getFPGATimestamp()

        cur_heading = self.vision_offset.heading
        cur_distance = self.vision_offset.y

        if (
            abs(heading_error) <= self.HEADING_TOLERANCE
            and abs(distance_error) <= self.DISTANCE_TOLERANCE
        ):
            self.next_state("startShooter")
        else:
            self.cur_time = Timer.getFPGATimestamp()
            dt = self.cur_time - self.last_time

            heading_adjust = self.heading_pidf.update(heading_error, dt)
            distance_adjust = self.distance_pidf.update(distance_error, dt)
            
            left_velocity = distance_adjust - heading_adjust
            right_velocity = distance_adjust + heading_adjust

            self.chassis.setVelocity(left_velocity, right_velocity)
            self.last_time = self.cur_time
            self.next_state("steerTowardTarget")

    # @state()
    # def adjustDistance(self, initial_call):
    #     distance_error = self.vision_offset.y
    #     if abs(distance_error) > self.DISTANCE_TOLERANCE:
    #         if initial_call:
    #             self.last_time = Timer.getFPGATimestamp()
    #         self.cur_time = Timer.getFPGATimestamp()
    #         dt = self.cur_time - self.last_time
    #         out = self.distance_pidf.update(dist_error, dt)
    #         self.chassis.setVelocity(out)
    #         self.last_time = self.cur_time
    #         self.next_state("adjustDistance")
    #     else:
    #         self.next_state("shootBalls")

    @timed_state(duration=3)
    def startShooter(self, initial_call, next_state="feedBalls"):
        if initial_call:
            self.chassis.stop()
            self.chassis.setBrakeMode()
            self.shooter.shoot()

    @timed_state(duration=10)
    def feedBalls(self, initial_call, next_state="success"):
        if initial_call:
            self.tower.runHighTower()
            self.tower.runLowTower()

    @state()
    def success(self):
        self.tower.stop()
        self.shooter.stop()
        self.chassis.setCoastMode()
        self.turret.stop()
        self.done()
