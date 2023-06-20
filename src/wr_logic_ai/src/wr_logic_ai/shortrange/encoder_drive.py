#!/usr/bin/env python

import math
from typing import Tuple

import rospy
from std_msgs.msg import Float64

from shortrange_util import ShortrangeStateEnum, ShortrangeState
from wr_logic_ai.shortrange.shortrange_util import ShortrangeStateEnum
from wrevolution.srv import ResetEncoder
from wr_drive_msgs.msg import DriveTrainCmd

# Base speed for robot to move at
SPEED = 0.1
# Diameter of robot wheels
ROBOT_WHEEL_DIAMETER = 1
# Encoder counts per rotation
ENCODER_COUNTS_PER_ROTATION = 2048


left_encoder_reset_srv = '/left/reset_encoder'
right_encoder_reset_srv = '/right/reset_encoder'

left_encoder_topic = '/left/encoder'
right_encoder_topic = '/right/encoder'

# TODO (@bennowotny @co-li ) This should be consolidated with gate navigation to avoid duplication
drivetrain_topic = rospy.get_param("~motor_speeds")
drivetrain_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)


def drive(left: float, right: float):
    drivetrain_pub.publish(left, right)


def distance_to_encoder(meters: float) -> int:
    # TODO (@bennowotny @co-li ) Math here is suspicious
    return int(ENCODER_COUNTS_PER_ROTATION * meters / (ROBOT_WHEEL_DIAMETER / 2 * math.pi))


class EncoderDrive(ShortrangeState):
    def __init__(self, distance: float) -> None:
        self.setpoint = distance_to_encoder(distance)
        self.is_done = [False] * 2
        self._cached_left_spd = 0
        self._cached_right_spd = 0

    def reset_encoder(self, srv_name: str):
        rospy.wait_for_service(srv_name)
        reset_encoder_srv = rospy.ServiceProxy(srv_name, ResetEncoder)
        reset_encoder_srv()

    def encoder_callback(self, setpoint: int, done_idx: int, encoder: Float64):
        if (encoder.data < setpoint):
            if done_idx == 0:
                self._cached_left_spd = SPEED
            else:
                self._cached_right_spd = SPEED
        else:
            self.is_done[done_idx] = True
            if done_idx == 0:
                self._cached_left_spd = 0
            else:
                self._cached_right_spd = 0
        drive(self._cached_left_spd, self._cached_right_spd)

    def run(self) -> Tuple[ShortrangeStateEnum, int]:
        rate = rospy.Rate(10)

        self.reset_encoder(left_encoder_reset_srv)
        self.reset_encoder(right_encoder_reset_srv)
        sub_left = rospy.Subscriber(left_encoder_topic, Float64, lambda msg: self.encoder_callback(
            self.setpoint, 0, msg))
        sub_right = rospy.Subscriber(right_encoder_topic, Float64, lambda msg: self.encoder_callback(
            self.setpoint, 1, msg))

        while not all(self.is_done) and not rospy.is_shutdown():
            rospy.sleep(rate)

        sub_left.unregister()
        sub_right.unregister()

        if all(self.is_done):
            return ShortrangeStateEnum.SUCCESS, 0
        return ShortrangeStateEnum.FAIL, 0
