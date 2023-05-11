#!/usr/bin/env python

from typing import Dict, Tuple
import rospy

from shortrange_util import ShortrangeStateEnum, ShortrangeState, TargetCache
from wr_logic_ai.msg import TargetMsg
from wr_drive_msgs.msg import DriveTrainCmd
from wr_logic_ai.src.shortrange_util import ShortrangeStateEnum

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


def drive(left: float, right: float):
    drivetrain_pub.publish(left, right)


class VisionNavigationGate(ShortrangeState):
    def __init__(self) -> None:
        self.is_done = False
        self.success = False
        self.distance = 0
        self.two_target_cache: Dict[int, TargetCache] = {}

    def gate_callback(self, msg: TargetMsg):
        if msg.valid:
            self.two_target_cache[msg.id] = TargetCache(rospy.get_time(), msg)
        if self.two_target_cache:
            if len(self.two_target_cache) == 2:
                # both targets found
                # estimate when gate is reached
                left, right = False, False
                avg_distance = 0
                for target in self.two_target_cache.values():
                    if target.msg.x_offset < 0:
                        left = True
                    elif target.msg.x_offset > 0:
                        right = True
                if left and right:
                    self.is_done = True
                    self.success = True
                    self.distance = avg_distance
                elif left:
                    drive(-SPEED, SPEED)
                elif right:
                    drive(SPEED, -SPEED)
            else:
                # only one target found
                drive(SPEED, -SPEED)
        else:
            # TODO search pattern
            drive(SPEED, -SPEED)
    
    def run(self) -> Tuple[ShortrangeStateEnum, int]:
        rate = rospy.Rate(10)
        sub = rospy.Subscriber(vision_topic, TargetCache, self.gate_callback)
        
        while not self.is_done:
            rospy.sleep(rate)

        sub.unregister()
        if self.success:
            return ShortrangeStateEnum.SUCCESS, self.distance
        return ShortrangeStateEnum.FAIL, 0
