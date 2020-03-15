import numpy as np
import wpilib
from magicbot.state_machine import StateMachine, state
from networktables import NetworkTables

from components import chassis, flywheel, turret, vision
from controls import pidf
from utils import drivesignal, lazypigeonimu, units


class AlignChassis(StateMachine):

    # target tracking pidf gains
    DISTANCE_KP = 4.5
    DISTANCE_KI = 0
    DISTANCE_KD = 0
    DISTANCE_KF = 0
    DISTANCE_MIN_OUTPUT = -1  # m / s
    DISTANCE_MAX_OUTPUT = 1  # m / s
    DISTANCE_TOLERANCE = 2 * units.meters_per_inch

    HEADING_KP = 1
    HEADING_KI = 0
    HEADING_KD = 0
    HEADING_KF = 0
    HEADING_MIN_OUTPUT = -0.4  # m / s
    HEADING_MAX_OUTPUT = 0.4  # m / s
    HEADING_TOLERANCE = 10 * units.radians_per_degree

    SEARCH_SPEED = 0.3

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
        self.desired_distance = 0

    def on_disable(self):
        self.done()

    def setup(self):
        self.distance_pidf = pidf.PIDF(
            self.DISTANCE_KP, self.DISTANCE_KI, self.DISTANCE_KD, self.DISTANCE_KF,
        )
        self.distance_pidf.setOutputRange(
            self.DISTANCE_MIN_OUTPUT, self.DISTANCE_MAX_OUTPUT
        )

        self.heading_pidf = pidf.PIDF(
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

        self.nt = NetworkTables.getTable("/components/alignchassis")

    def align(self):
        """Enable the statemachine."""
        self.engage()

    def isAligned(self):
        """Is the chassis at an ok distance and heading."""
        return (
            abs(self.desired_distance - self.vision.getDistance())
            <= self.DISTANCE_TOLERANCE
        ) and (abs(self.vision.getHeading()) <= self.HEADING_TOLERANCE)

    @state(first=True)
    def findTarget(self, initial_call):
        """Spin in a circle until a vision target is found."""
        if self.vision.hasTarget():
            self.next_state("driveToTarget")
        else:
            self.chassis.setOutput(self.SEARCH_SPEED, -self.SEARCH_SPEED)

    @state()
    def driveToTarget(self, initial_call):
        """Drive to the desired distance while adjusting heading."""
        if initial_call:
            # min_distance = self.flywheel.DISTANCES[0]
            # max_distance = np.max(np.where(self.flywheel.ACCURACY == 1))
            # self.desired_distance = np.clip(
            #     min_distance, max_distance, self.vision.getDistance(),
            # )
            self.desired_distance = 15 * units.meters_per_foot
            self.heading_pidf.setSetpoint(0)
            self.distance_pidf.setSetpoint(self.desired_distance)
            self.prev_time = wpilib.Timer.getFPGATimestamp()
            self.chassis.setCoastMode()

        cur_time = wpilib.Timer.getFPGATimestamp()
        # dt = cur_time - self.prev_time
        dt = 0.02

        # calculate pidf outputs
        distance = self.vision.getDistance()
        heading = self.vision.getHeading()

        self.distance_adjust = -self.distance_pidf.update(distance, dt)
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
        """Set brake mode to lock chassis in place."""
        if initial_call:
            self.chassis.setBrakeMode()
        if not self.isAligned():
            self.next_state("driveToTarget")
        else:
            self.chassis.stop()

    def done(self):
        super().done()
        self.chassis.setCoastMode()
        self.chassis.stop()
        self.vision.enableLED(False)

    def updateNetworkTables(self):
        self.nt.putNumber("desired_velocity_left", self.desired_velocity.left)
        self.nt.putNumber("desired_velocity_right", self.desired_velocity.right)
        self.nt.putNumber("distance_adjust", self.distance_adjust)
        self.nt.putNumber("heading_adjust", self.heading_adjust)

    def execute(self):
        super().execute()
        self.updateNetworkTables()
        if self.is_executing:
            self.vision.enableLED(True)
