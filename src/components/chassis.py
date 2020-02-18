from wpilib import Timer, controller
from enum import Enum
from utils import (
    lazypigeonimu,
    lazytalonfx,
    units,
    drivesignal,
    wheelstate,
)
from controls import pidf
from components import vision
import numpy as np
from networktables import NetworkTables


class Chassis:

    # chassis physical constants
    TRACK_WIDTH = 24 * units.meters_per_inch
    TRACK_RADIUS = TRACK_WIDTH / 2

    WHEEL_DIAMETER = 6 * units.meters_per_inch
    WHEEL_RADIUS = WHEEL_DIAMETER / 2
    WHEEL_CIRCUMFERENCE = 2 * np.pi * WHEEL_RADIUS
    GEAR_RATIO = (48 / 14) * (50 / 16)  # 10.7142861

    # conversions
    RADIANS_PER_METER = (np.pi * 2 * GEAR_RATIO) / WHEEL_CIRCUMFERENCE  # rad / m
    METERS_PER_RADIAN = WHEEL_CIRCUMFERENCE / (np.pi * 2 * GEAR_RATIO)  # m / rad

    # motor config
    TIMEOUT = lazytalonfx.LazyTalonFX.TIMEOUT
    LEFT_INVERTED = True
    RIGHT_INVERTED = False
    STATUS_FRAME = 10

    CLOSED_LOOP_RAMP = 0.5
    OPEN_LOOP_RAMP = 0.5

    STATOR_CURRENT = 40
    STATOR_TRIGGER = 60
    STATOR_TIME = 1

    SUPPLY_CURRENT = 40
    SUPPLY_TRIGGER = 60
    SUPPLY_TIME = 1

    # motor coefs
    DRIVE_KS = 0.149  # V
    DRIVE_KV = 2.4  # V / (m / s)
    DRIVE_KA = 0.234  # V / (m / s^2)

    # velocity pidf gains
    VELOCITY_LEFT_KP = 0.000363  # LQR = 0.000363
    VELOCITY_LEFT_KI = 0
    VELOCITY_LEFT_KD = 0
    VELOCITY_LEFT_KF = 0

    VELOCITY_RIGHT_KP = 0.000363  # LQR = 0.000363
    VELOCITY_RIGHT_KI = 0
    VELOCITY_RIGHT_KD = 0
    VELOCITY_RIGHT_KF = 0

    # target tracking pidf gains
    HEADING_KP = 0
    HEADING_KI = 0
    HEADING_KD = 0
    HEADING_KF = 0

    DISTANCE_KP = 0
    DISTANCE_KI = 0
    DISTANCE_KD = 0
    DISTANCE_KF = 0

    # joystick control parameters
    JOYSTICK_THROTTLE_EXPONENT = 1
    JOYSTICK_ROTATION_EXPONENT = 1
    JOYSTICK_MAX_PERCECNT = 0.5
    JOYSTICK_DEADBAND = 0.08

    # required components
    vision: vision.Vision

    # required devices
    drive_master_left: lazytalonfx.LazyTalonFX
    drive_master_right: lazytalonfx.LazyTalonFX

    imu: lazypigeonimu.LazyPigeonIMU

    class _Mode(Enum):
        Idle = 0
        PercentOutput = 1
        Velocity = 2
        Vision = 3

    def __init__(self):
        self.mode = self._Mode.Idle

        self.desired_output = drivesignal.DriveSignal()
        self.desired_velocity = drivesignal.DriveSignal()
        self.desired_acceleration = drivesignal.DriveSignal()
        self.feedforward = drivesignal.DriveSignal()
        self.prev_time = 0
        self.wheel_position = wheelstate.WheelState()
        self.prev_wheel_position = wheelstate.WheelState()
        self.wheel_velocity = wheelstate.WheelState()
        self.heading = 0
        self.nt = NetworkTables.getTable(
            f"/components/{self.__class__.__name__.lower()}"
        )

    # TODO delete this function
    def setupPIDF(self):
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
        self.distance_pidf = pidf.PIDF(
            self.vision.DISTANCE_DESIRED,
            self.DISTANCE_KP,
            self.DISTANCE_KI,
            self.DISTANCE_KD,
            self.DISTANCE_KF,
        )
        self.drive_master_left.setPIDF(
            0,
            self.VELOCITY_LEFT_KP,
            self.VELOCITY_LEFT_KI,
            self.VELOCITY_LEFT_KD,
            self.VELOCITY_LEFT_KF,
        )
        self.drive_master_right.setPIDF(
            0,
            self.VELOCITY_RIGHT_KP,
            self.VELOCITY_RIGHT_KI,
            self.VELOCITY_RIGHT_KD,
            self.VELOCITY_RIGHT_KF,
        )

    def setup(self):
        self.drive_master_left.setInverted(self.LEFT_INVERTED)
        self.drive_master_right.setInverted(self.RIGHT_INVERTED)

        for master in (self.drive_master_left, self.drive_master_right):
            master.setStatusFramePeriod(
                lazytalonfx.LazyTalonFX.StatusFrame.Status_2_Feedback0,
                self.STATUS_FRAME,
                self.TIMEOUT,
            )
            master.configOpenloopRamp(self.OPEN_LOOP_RAMP, self.TIMEOUT)
            master.configClosedloopRamp(self.CLOSED_LOOP_RAMP, self.TIMEOUT)

            master.setStatorCurrentLimit(
                self.STATOR_CURRENT, self.STATOR_TRIGGER, self.STATOR_TIME
            )
            master.setSupplyCurrentLimit(
                self.SUPPLY_CURRENT, self.SUPPLY_TRIGGER, self.SUPPLY_TIME
            )

        # drive motor characterizations
        self.drive_characterization_left = controller.SimpleMotorFeedforwardMeters(
            self.DRIVE_KS, self.DRIVE_KV, self.DRIVE_KA,
        )
        self.drive_characterization_right = controller.SimpleMotorFeedforwardMeters(
            self.DRIVE_KS, self.DRIVE_KV, self.DRIVE_KA,
        )

        self.setupPIDF()

    def on_enable(self):
        # TODO remove setup pidf from enable
        self.setupPIDF()

    def on_disable(self):
        self.stop()

    def isAligning(self):
        return self.mode == self._Mode.Vision

    def setOutput(self, output_l: float, output_r: float) -> None:
        """Set the output of the motors."""
        self.mode = self._Mode.PercentOutput
        self.desired_output.left = output_l
        self.desired_output.right = output_r

    def setFromJoystick(self, throttle: float, rotation: float) -> None:
        """Set the output of the motors from joystick values."""
        throttle **= self.JOYSTICK_THROTTLE_EXPONENT
        rotation **= self.JOYSTICK_ROTATION_EXPONENT

        throttle = 0 if abs(throttle) <= self.JOYSTICK_DEADBAND else throttle
        rotation = 0 if abs(rotation) <= self.JOYSTICK_DEADBAND else rotation

        output_l = np.clip(
            throttle - rotation, -self.JOYSTICK_MAX_PERCECNT, self.JOYSTICK_MAX_PERCECNT
        )
        output_r = np.clip(
            throttle + rotation, -self.JOYSTICK_MAX_PERCECNT, self.JOYSTICK_MAX_PERCECNT
        )
        self.setOutput(output_l, output_r)

    def setVelocity(self, velocity_l: float, velocity_r: float) -> None:
        """Set the velocity of the motors in m/s."""
        self.mode = self._Mode.Velocity
        self.desired_velocity.left = velocity_l
        self.desired_velocity.right = velocity_r

    def setRotationalVelocity(self, velocity: float) -> None:
        """Set the rotational velocity of the chassis in rad/s."""
        velocity_l = -velocity * self.TRACK_RADIUS
        velocity_r = velocity * self.TRACK_RADIUS
        self.setVelocity(velocity_l, velocity_r)

    def trackTarget(self) -> None:
        """Track the vision target and drive to it."""
        self.mode = self._Mode.Vision

    def stop(self) -> None:
        """Stop all motor output."""
        self.mode = self._Mode.Idle

    def setBreakMode(self) -> None:
        """Set the motors to break mode."""
        self.drive_master_left.setBreakMode()
        self.drive_master_right.setBreakMode()

    def setCoastMode(self) -> None:
        """Set the motors to coast mode."""
        self.drive_master_left.setCoastMode()
        self.drive_master_right.setCoastMode()

    def _computeFF(self) -> drivesignal.DriveSignal:
        self.feedforward.left = (
            self.drive_characterization_left.calculate(
                self.desired_velocity.left, self.desired_acceleration.left
            )
            / 12
        )
        self.feedforward.right = (
            self.drive_characterization_right.calculate(
                self.desired_velocity.right, self.desired_acceleration.right
            )
            / 12
        )

    def _computeAcceleration(self, dt: float):
        self.desired_acceleration.left = (
            self.desired_velocity.left - self.wheel_velocity.left
        ) / dt
        self.desired_acceleration.right = (
            self.desired_velocity.right - self.wheel_velocity.right
        ) / dt

    def updateNetworkTables(self):
        self.nt.putValue("wheel_position_left", self.wheel_position.left)
        self.nt.putValue("wheel_position_right", self.wheel_position.right)
        self.nt.putValue("wheel_velocity_left", self.wheel_velocity.left)
        self.nt.putValue("wheel_velocity_right", self.wheel_velocity.right)

        self.nt.putValue("desired_velocity_left", self.desired_velocity.left)
        self.nt.putValue("desired_velocity_right", self.desired_velocity.right)
        self.nt.putValue(
            "error_velocity_left", self.desired_velocity.left - self.wheel_velocity.left
        )
        self.nt.putValue(
            "error_velocity_right",
            self.desired_velocity.right - self.wheel_velocity.right,
        )
        self.nt.putValue("feedforward_left", self.feedforward.left)
        self.nt.putValue("feedforward_right", self.feedforward.right)
        self.nt.putValue("heading", self.heading * units.degrees_per_radian)

    def execute(self):
        cur_time = Timer.getFPGATimestamp()
        # dt = cur_time - self.prev_time
        dt = 0.02

        self.wheel_position.left = (
            self.drive_master_left.getPosition() * self.METERS_PER_RADIAN
        )
        self.wheel_position.right = (
            self.drive_master_right.getPosition() * self.METERS_PER_RADIAN
        )

        self.wheel_velocity.left = (
            self.wheel_position.left - self.prev_wheel_position.left
        ) / dt
        self.wheel_velocity.right = (
            self.wheel_position.right - self.prev_wheel_position.right
        ) / dt

        self.heading = self.imu.getHeadingInRange()

        if self.mode == self._Mode.Idle:
            # stop the motors
            self.drive_master_left.setOutput(0.0)
            self.drive_master_right.setOutput(0.0)
        elif self.mode == self._Mode.PercentOutput:
            # set the output of the motors
            self.drive_master_left.setOutput(self.desired_output.left)
            self.drive_master_right.setOutput(self.desired_output.right)
        elif self.mode == self._Mode.Velocity:
            # set the velocity of the motors
            # self.desired_velocity.left = self.DESIRED_TEST
            # self.desired_velocity.right = self.DESIRED_TEST

            self._computeAcceleration(dt)
            self._computeFF()

            self.drive_master_left.setVelocity(
                self.desired_velocity.left * self.RADIANS_PER_METER,
                self.feedforward.left,
            )
            self.drive_master_right.setVelocity(
                self.desired_velocity.right * self.RADIANS_PER_METER,
                self.feedforward.right,
            )
        elif self.mode == self._Mode.Vision:
            # calculate pidf outputs
            distance_error = self.vision.getDistanceError()
            heading_error = self.vision.getHeadingError()

            distance_adjust = self.distance_pidf.update(distance_error, dt)
            heading_adjust = self.heading_pidf.update(heading_error, dt)

            # calculate wheel velocities and set motor outputs
            self.desired_velocity.left = distance_adjust + heading_adjust
            self.desired_velocity.right = distance_adjust - heading_adjust

            self._computeAcceleration(dt)
            self._computeFF()

            self.drive_master_left.setVelocity(
                self.desired_velocity.left * self.RADIANS_PER_METER,
                self.feedforward.left,
            )
            self.drive_master_right.setVelocity(
                self.desired_velocity.right * self.RADIANS_PER_METER,
                self.feedforward.right,
            )
        self.prev_time = cur_time

        self.prev_wheel_position.left = self.wheel_position.left
        self.prev_wheel_position.right = self.wheel_position.right

        self.updateNetworkTables()
