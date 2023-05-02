#!/usr/bin/env python

import math

import rospy
from std_msgs.msg import Float64

from shortrange_util import ShortrangeAIStates, ShortrangeState
from wr_logic_ai.src.shortrange_util import ShortrangeAIStates
from wrevolution.srv import ResetEncoder

# Base speed for robot to move at
SPEED = 0.1
# Diameter of robot wheels
ROBOT_WHEEL_DIAMETER = 1
# Encoder counts per rotation
ENCODER_COUNTS_PER_ROTATION = 2048


left_encoder_reset_srv = ''
right_encoder_reset_srv = ''

left_motor_topic = ''
right_motor_topic = ''

left_encoder_topic = ''
right_encoder_topic = ''

left_motor_pub = rospy.Publisher(left_motor_topic, Float64)
right_motor_pub = rospy.Publisher(right_motor_topic, Float64)


def distance_to_encoder(meters: float) -> int:
    return int(ENCODER_COUNTS_PER_ROTATION * meters / (ROBOT_WHEEL_DIAMETER / 2 * math.pi))


class EncoderDrive(ShortrangeState):
    def __init__(self, distance: float) -> None:
        self.is_done = False
        self.setpoint = distance_to_encoder(distance)

    def reset_encoder(self, srv_name: str):
        rospy.wait_for_service(srv_name)
        reset_encoder_srv = rospy.ServiceProxy(srv_name, ResetEncoder)
        reset_encoder_srv()

    def encoder_callback(self, motor_pub: rospy.Publisher, setpoint: int, encoder: Float64):
        if (encoder.data < setpoint):
            motor_pub.publish(Float64(SPEED)) 
        else:
            motor_pub.publish(Float64(0))

    def run(self) -> ShortrangeAIStates:
        rate = rospy.Rate(10)

        self.reset_encoder(left_encoder_reset_srv)
        self.reset_encoder(right_encoder_reset_srv)
        sub_left = rospy.Subscriber(left_encoder_topic, Float64, lambda msg : self.encoder_callback(left_motor_pub, self.setpoint, msg))
        sub_right = rospy.Subscriber(right_encoder_topic, Float64, lambda msg: self.encoder_callback(right_motor_pub, self.setpoint, msg))

        while not self.is_done:
            rospy.sleep(rate)
        
        sub_left.unregister()
        sub_right.unregister()

        return ShortrangeAIStates.FAIL
