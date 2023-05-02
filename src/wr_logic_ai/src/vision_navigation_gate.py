#!/usr/bin/env python

import rospy

from shortrange_util import ShortrangeAIStates, ShortrangeState, TargetCache
from wr_logic_ai.msg import TargetMsg
from wr_drive_msgs.msg import DriveTrainCmd

# TODO : Maybe use ROS params/remaps (low priority, can do later)
vision_topic = rospy.get_param('vision_topic')

# TODO : Add code for mux
drivetrain_topic = '/control/drive_system/cmd'
drivetrain_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)


def drive(left: float, right: float):
    drivetrain_pub.publish(left, right)


class VisionNavigationGate(ShortrangeState):
    def __init__(self) -> None:
        self.two_target_cache = {}
        self.is_done = False
        self.success = False

    def gate_callback(self, msg: TargetMsg):
        # TODO handle two targets
        if msg.valid:
            self.two_target_cache[msg.id] = TargetCache(rospy.get_time(), msg)
        if self.two_target_cache:
            if len(self.two_target_cache) == 2:
                # both targets found
                # estimate when gate is reached
                pass
            else:
                # only one target found
                pass
        else:
            # TODO search pattern
            drive(0, 0)
    
    def run(self) -> ShortrangeAIStates:
        rate = rospy.Rate(10)
        sub = rospy.Subscriber(vision_topic, TargetCache, self.gate_callback)
        
        while not self.is_done:
            rospy.sleep(rate)

        sub.unregisters()
        if self.success:
            return ShortrangeAIStates.SUCCESS
        return ShortrangeAIStates.FAIL
