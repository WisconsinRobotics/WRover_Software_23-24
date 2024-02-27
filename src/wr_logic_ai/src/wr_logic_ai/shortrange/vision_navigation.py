#!/usr/bin/env python

##@defgroup wr_shortrange_ai
# @{
# @defgroup wr_shortrange_ai_navigation Vision Navigation
# @brief ShortrangeState that subscribes to vision data and drives to the ArUco tag
# @{

from typing import *

import rospy

from shortrange_util import ShortrangeStateEnum, ShortrangeState, TargetCache
from wr_logic_ai.msg import VisionTarget
from wr_drive_msgs.msg import DriveTrainCmd
from wr_logic_ai.shortrange.shortrange_util import ShortrangeStateEnum

## Distance from target to stop at (in meters)
STOP_DISTANCE_M = 1.5
## Base speed for robot to move at
SPEED = 0.1
## Factor to for applying turn
kP = 0.01

# Number of seconds to keep the cache
CACHE_EXPIRY_SECS = 3


class VisionNavigation(ShortrangeState):
    """
    A ShortrangeState class for navigating to a ArUco tag
    """

    def __init__(self) -> None:
        ## Boolean for state progress.
        ## True if the state is done, false otherwise.
        self.is_done = False
        ## Boolean to state result or failure.
        ## True if the state is successful, false otherwise.
        self.success = False
        ## TargetCache variable to store the last seen target data
        self.target_cache = None

    def target_callback(self, msg: VisionTarget):
        if msg.valid:
            # Update cache if the VisionTarget message is valid
            self.target_cache = TargetCache(rospy.get_time(), msg)

    def run(self) -> ShortrangeStateEnum:
        """
        Run target navigation logic.
        Creates a subscriber for the VisionTarget topic.

        @return ShortrangeStateEnum: The next state to execute
        """
        rate = rospy.Rate(10)

        # Name of the VisionTarget topic to subscribe to
        vision_topic = rospy.get_param("~vision_topic")
        sub = rospy.Subscriber(vision_topic, VisionTarget, self.target_callback)

        # Name of the drivetrain topic to publish to
        drivetrain_topic = rospy.get_param("~motor_speeds")
        # Publisher to set motor speeds
        drivetrain_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)

        # Wait for the rover to finish navigating
        while not self.is_done:

            if (
                self.target_cache is not None
                and rospy.get_time() - self.target_cache.timestamp < CACHE_EXPIRY_SECS
            ):
                if self.target_cache.msg.distance_estimate < STOP_DISTANCE_M:
                    # Stop the rover when it is close to the ArUco tag
                    rospy.loginfo(f"Reached target {self.target_cache.msg.id}")
                    drivetrain_pub.publish(0, 0)

                    self.is_done = True
                    self.success = True
                else:
                    # Drive the rover to the target if the cache was updated recently
                    turn = kP * self.target_cache.msg.x_offset
                    drivetrain_pub.publish(SPEED + turn, SPEED - turn)
            else:
                # Turn the rover to look for the ArUco tag
                drivetrain_pub.publish(SPEED, -SPEED)

            rospy.sleep(rate)

        sub.unregister()
        drivetrain_pub.unregister()

        if self.success:
            return ShortrangeStateEnum.SUCCESS
        return ShortrangeStateEnum.FAIL


## @}
# @}
