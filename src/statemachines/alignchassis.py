import numpy as np
from networktables import NetworkTables

import wpilib
from components import chassis, turret, vision, flywheel
from controls import pidf
from magicbot.state_machine import StateMachine, state
from utils import drivesignal, lazypigeonimu, units


class AlignChassis(StateMachine):

    # target tracking pidf gains
    DISTANCE_KP = 4.5
    DISTANCE_KI = 0
    DISTANCE_KD = 0
    DISTANCE_KF = 0
    DISTANCE_MIN_OUTPUT = -1  # m / s
    DISTANCE_MAX_OUTPUT = 1  # m / s
    DISTANCE_DESIRED = 144 * units.meters_per_inch
    DISTANCE_TOLERANCE = 1 * units.meters_per_inch

    HEADING_KP = 1
    HEADING_KI = 0
    HEADING_KD = 0
    HEADING_KF = 0
    HEADING_MIN_OUTPUT = -0.4  # m / s
    HEADING_MAX_OUTPUT = 0.4  # m / s
    HEADING_TOLERANCE = 10 * units.radians_per_degree

    TURN_KP = 0
    TURN_KI = 0
    TURN_KD = 0
    TURN_KF = 0
    TURN_MIN_OUTPUT = -90 * units.radians_per_degree  # rad / s
    TURN_MAX_OUTPUT = 90 * units.radians_per_degree  # rad / s
    TURN_TOLERANCE = 1 * units.radians_per_degree  # rad / s

    SEARCH_SPEED = 0.1

    chassis: chassis.Chassis
    turret: turret.Turret
    vision: vision.Vision
    imu: lazypigeonimu.LazyPigeonIMU
    flywheel: flywheel.Flywheel

    def __init__(self):
        self.desired_velocity = drivesignal.DriveSignal()
        self.distance_adjust = 0
        self.heading_adjust = 0
        self.prev_time = 0

    def setup(self):
        self.distance_pidf = pidf.PIDF(
            self.DISTANCE_DESIRED,
            self.DISTANCE_KP,
            self.DISTANCE_KI,
            self.DISTANCE_KD,
            self.DISTANCE_KF,
        )
        self.distance_pidf.setOutputRange(
            self.DISTANCE_MIN_OUTPUT, self.DISTANCE_MAX_OUTPUT
        )
        self.heading_pidf = pidf.PIDF(
            0,
            self.HEADING_KP,
            self.HEADING_KI,
            self.HEADING_KD,
            self.HEADING_KF,
            True,
            -np.pi,
            np.pi,
        )
        self.heading_pidf.setOutputRange(
            self.HEADING_MIN_OUTPUT, self.HEADING_MAX_OUTPUT
        )
        self.turn_pidf = pidf.PIDF(
            0,
            self.TURN_KP,
            self.TURN_KI,
            self.TURN_KD,
            self.TURN_KF,
            True,
            -np.pi,
            np.pi,
        )
        self.turn_pidf.setOutputRange(self.TURN_MIN_OUTPUT, self.TURN_MAX_OUTPUT)

        self.nt = NetworkTables.getTable("/components/alignchassis")

    def align(self):
        self.engage()

    # def getAllocentricHeading(self):
    #     return (
    #         self.vision.getHeading() - self.turret.getHeading() - self.imu.getHeading()
    #     )

    # def getComponentDistanceToPort(self):
    #     heading = self.vision.getHeading()
    #     distance = self.vision.getDistance()
    #     y = np.sin(np.pi - heading) * distance
    #     x = np.sqrt(distance ** 2 - y ** 2)
    #     return (x, y)

    # def getAlignmentTargetDistance(self):
    #     heading = self.imu.getHeadingInRange()
    #     x, _ = self.getComponentDistanceToPort()
    #     target = (x * np.sin(np.pi - heading)) / np.sin(heading)
    #     return np.clip(self.flywheel.DISTANCES[0], self.flywheel.DISTANCES[-1], target)

    # def getComponentDistanceToAlignmentTarget(self, target):
    #     heading = self.getAllocentricHeading()
    #     x = self.vision.getDistance() * np.sin(heading)
    #     distance = self.vision.getDistance()
    #     y = target - np.sqrt(distance ** 2 - x ** 2)
    #     return (x, y)

    # def getDistanceToAlignmentTarget(self, target):
    #     x, y = self.getComponentDistanceToAlignmentTarget(target)
    #     return np.sqrt(x ** 2 + y ** 2)

    # def getAngleToAlignmentTarget(self, target):
    #     x, _ = self.getComponentDistanceToAlignmentTarget(target)
    #     distance = self.getDistanceToAlignmentTarget(target)
    #     alpha = np.arcsin(x / distance)
    #     return np.pi - alpha - self.imu.getHeadingInRange()

    def isAligned(self):
        return (
            abs(self.DISTANCE_MIN_OUTPUT - self.vision.getDistance())
            <= self.DISTANCE_TOLERANCE
        ) and (abs(self.vision.getHeading()) <= self.HEADING_TOLERANCE)

    # @state()
    # def turnToHeading(self, initial_call):
    #     if initial_call:
    #         self.turn_pidf.reset()
    #         self.turn_pidf.setpoint = desired_heading
    #     dt = 0.02
    #     cur_time = wpilib.Timer.getFPGATimestamp()
    #     heading =  self.imu.getHeadingInRange()
    #     if abs(units.angle_diff(heading,desired_heading)) > self.TURN_TOLERANCE:
    #         omega = self.turn_pidf.update(dt, self.imu.getHeadingInRange())
    #         self.chassis.setRotationalVelocity(omega)
    #     else:
    #         self.next_state(_next_state)
    #     self.prev_time = cur_time

    @state(first=True)
    def findTarget(self):
        if self.vision.hasTarget():
            self.next_state("driveToTarget")
        else:
            self.chassis.setOutput(self.SEARCH_SPEED, -self.SEARCH_SPEED)

    @state()
    def driveToTarget(self, initial_call):
        if initial_call:
            self.prev_time = wpilib.Timer.getFPGATimestamp()
            self.chassis.setCoastMode()

        cur_time = wpilib.Timer.getFPGATimestamp()
        # dt = cur_time - self.prev_time
        dt = 0.02

        # calculate pidf outputs
        distance = self.vision.getDistance()
        heading = self.vision.getHeading()

        self.distance_adjust = self.distance_pidf.update(distance, dt)
        self.heading_adjust = self.heading_pidf.update(heading, dt)

        # calculate wheel velocities and set motor outputs
        self.desired_velocity.left = self.distance_adjust + self.heading_adjust
        self.desired_velocity.right = self.distance_adjust - self.heading_adjust

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

    def execute(self):
        super().execute()
        self.nt.putNumber(
            "allocentric_heading",
            self.getAllocentricHeading() * units.degrees_per_radian,
        )
        self.nt.putNumber("desired_velocity_left", self.desired_velocity.left)
        self.nt.putNumber("desired_velocity_right", self.desired_velocity.right)
        self.nt.putNumber("distance_adjust", self.distance_adjust)
        self.nt.putNumber("heading_adjust", self.heading_adjust)

        # cx, cy = self.getComponentDistanceToPort()

        # self.nt.putNumber("component_port_x", cx * units.inches_per_meter)
        # self.nt.putNumber("component_port_y", cy * units.inches_per_meter)
        # self.nt.putNumber(
        #     "alignment_target",
        #     self.getAlignmentTargetDistance() * units.inches_per_meter,
        # )

