#!/usr/bin/env python

import math
from enum import Enum
from typing import Tuple, Union

import rospy
from geometry_msgs.msg import PoseStamped

from shortrange_util import ShortrangeStateEnum, ShortrangeState, TargetCache
from wr_logic_ai.msg import VisionTarget
from wr_drive_msgs.msg import DriveTrainCmd
from wr_logic_ai.shortrange.shortrange_util import ShortrangeStateEnum

# TODO : Document/rename variables, I'm not sure what all of these are for

# distance from target to stop at (in meters)
STOP_DISTANCE_M = 3
# Base speed for robot to move at
SPEED = 0.1
# Factor to for applying turn
kP = 0.01

# TODO : Maybe use ROS params/remaps (low priority, can do later)
vision_topic = rospy.get_param('vision_topic')

# TODO : Add code for mux
drivetrain_topic = '/control/drive_system/cmd'
drivetrain_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)

CACHE_EXPIRY_SECS = 1


def drive(left: float, right: float):
    drivetrain_pub.publish(left, right)


class VisionNavigationPost(ShortrangeState):
    def __init__(self) -> None:
        self.is_done = False
        self.success = False
        self.target_cache: Union[TargetCache, None] = None

    def target_callback(self, msg: VisionTarget):
        if msg.valid:
            self.target_cache = TargetCache(rospy.get_time(), msg)
        if self.target_cache is not None and rospy.get_time() - self.target_cache.timestamp < CACHE_EXPIRY_SECS:

            if self.target_cache.msg.distance_estimate < STOP_DISTANCE_M:
                rospy.loginfo(f'Reached target {self.target_cache.msg.id}')
                drive(0, 0)

                self.success = True
                return

            turn = kP * self.target_cache.msg.x_offset
            drive(SPEED + turn, SPEED - turn)
        else:
            drive(SPEED, -SPEED)

    def run(self) -> Tuple[ShortrangeStateEnum, int]:
        rate = rospy.Rate(10)

        sub = rospy.Subscriber(
            vision_topic, VisionTarget, self.target_callback)

        while not self.is_done:
            rospy.sleep(rate)

        sub.unregister()
        if self.success:
            return ShortrangeStateEnum.SUCCESS, 0
        return ShortrangeStateEnum.FAIL, 0
