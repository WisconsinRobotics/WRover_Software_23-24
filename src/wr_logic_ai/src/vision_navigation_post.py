#!/usr/bin/env python

import math
from enum import Enum
from typing import Tuple

import rospy
from geometry_msgs.msg import PoseStamped

from shortrange_util import ShortrangeStateEnum, ShortrangeState, TargetCache
from wr_logic_ai.msg import TargetMsg
from wr_drive_msgs.msg import DriveTrainCmd
from wr_logic_ai.src.shortrange_util import ShortrangeStateEnum

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


# Publish to topics for rviz if debug is turned on in vision_params
# debug = rospy.get_param('debug', False)
# if debug:
#     rviz_zero_topic = '/debug_zero'
#     rviz_zero = rospy.Publisher(rviz_zero_topic, PoseStamped, queue_size=10)

#     rviz_topic = '/debug_vision'
#     rviz_pub = rospy.Publisher(rviz_topic, PoseStamped, queue_size=10)

#     zero_msg = PoseStamped()
#     zero_msg.pose.position.x = 0
#     zero_msg.pose.position.y = 0
#     zero_msg.pose.position.z = 0
#     zero_msg.pose.orientation.x = 0
#     zero_msg.pose.orientation.y = 0
#     zero_msg.pose.orientation.z = 0
#     zero_msg.pose.orientation.w = 1
#     zero_msg.header.frame_id = 'map'
#     zero_msg.header.seq = 0

#     pose_msg = PoseStamped()
#     pose_msg.pose.position.x = 0
#     pose_msg.pose.position.y = 0
#     pose_msg.pose.position.z = 0
#     pose_msg.pose.orientation.x = 0
#     pose_msg.pose.orientation.y = 0
#     pose_msg.pose.orientation.z = 0
#     pose_msg.pose.orientation.w = 1
#     pose_msg.header.frame_id = 'map'
#     pose_msg.header.seq = 0


CACHE_EXPIRY_SECS = 1
target_cache = None


def drive(left: float, right: float):
    drivetrain_pub.publish(left, right)


class VisionNavigationPost(ShortrangeState):
    def __init__(self) -> None:
        self.is_done = False
        self.success = False

    def target_callback(self, msg: TargetMsg):
        global target_cache
        if msg.valid:
            target_cache = TargetCache(rospy.get_time(), msg)
        if target_cache is not None and rospy.get_time() - target_cache.timestamp < CACHE_EXPIRY_SECS:
            if target_cache.msg.distance_estimate < STOP_DISTANCE_M:
                rospy.loginfo(f'Reached target {target_cache.msg.id}')
                self.success = True
                drive(0, 0)
                return
            turn = kP * target_cache.msg.x_offset
            drive(SPEED + turn, SPEED - turn)
            # if debug:
            #     heading = (SPEED + turn) * -45 + (SPEED - turn) * 45
            #     pose_msg.pose.orientation.z = math.sin(math.radians(heading) / 2)
            #     pose_msg.pose.orientation.w = math.cos(math.radians(heading) / 2)
            #     rviz_zero.publish(zero_msg)
            #     rviz_pub.publish(pose_msg)
        else:
            drive(SPEED, -SPEED)
    
    def  run(self) -> Tuple[ShortrangeStateEnum, int]:
        rate = rospy.Rate(10)

        sub = rospy.Subscriber(vision_topic, TargetMsg, self.target_callback)

        while not self.is_done:
            rospy.sleep(rate)
        
        sub.unregister()
        if self.success:
            return ShortrangeStateEnum.SUCCESS, 0
        return ShortrangeStateEnum.FAIL, 0



# def gate_callback(msg: TargetMsg):
#     # TODO handle two targets
#     global two_target_cache
#     if msg.valid:
#         two_target_cache[msg.id] = TargetCache(rospy.get_time(), msg)
#     if two_target_cache:
#         if len(two_target_cache) == 2:
#             # both targets found
#             # estimate when gate is reached
#             pass
#         else:
#             # only one target found
#             pass
#     else:
#         # TODO search pattern
#         drive(0, 0)


# def main():
    # rospy.Subscriber(vision_topic, TargetMsg, target_callback)

    # rospy.spin()
