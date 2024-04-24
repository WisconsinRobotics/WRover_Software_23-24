#!/usr/bin/env python

##@defgroup wr_shortrange_ai
# @{
# @defgroup wr_shortrange_ai_action_server Shortrange Action Server
# @brief Shortrange action server 
# @details The shortrange action server drives the rover to a vision target.
#
# @{

from typing import *

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from wr_logic_shortrange.shortrange_util import ShortrangeStateEnum, ShortrangeState, TargetCache
from wr_logic_shortrange.vision_navigation import VisionNavigation
from wr_logic_shortrange.shortrange_util import ShortrangeStateEnum
from wr_logic_shortrange.msg import ShortRangeAction, ShortRangeGoal, VisionTarget
from wr_drive_msgs.msg import DriveTrainCmd

## Distance from target to stop at (in meters)
STOP_DISTANCE_M = 1.5
## Base speed for robot to move at
SPEED = 0.1
## Factor to for applying turn
kP = 0.01

# Number of seconds to keep the cache
CACHE_EXPIRY_SECS = 3

class ShortrangeActionServer:
    """
    @brief Shortrange ActionServer

    The shortrange state machine is implemented as an ActionServer.
    """

    def __init__(self, name: str) -> None:
        """
        Initialize the shortrange state machine

        @param name (str): The name of the action
        """
        ## The name of the action
        self._action_name = name
        ## SimpleActionServer using shortrange_callback to execute ShortrangeGoals
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ShortRangeAction,
            execute_cb=self.shortrange_callback,
            auto_start=False,
        )
        self._as.start()
        self.target_cache = None

    def target_callback(self, msg: VisionTarget):
        if msg.valid:
            # Update cache if the VisionTarget message is valid
            self.target_cache = TargetCache(rospy.get_time(), msg)

    def shortrange_callback(self, goal: ShortRangeGoal):
        """
        Sets the shortrange state based on the ShortRangeGoal message

        @param goal (ShortRangeGoal): ShortRangeGoal message defined in action/ShortRange.action
        """

        rate = rospy.Rate(10)

        # Name of the VisionTarget topic to subscribe to
        vision_topic = rospy.get_param("~vision_topic")
        sub = rospy.Subscriber(vision_topic, VisionTarget, self.target_callback)

        # Name of the drivetrain topic to publish to
        drivetrain_topic = rospy.get_param("~motor_speeds")
        # Publisher to set motor speeds
        drivetrain_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)

        success = False

        # Loop for running ArUco tag approach
        while not rospy.is_shutdown() and not self._as.is_preempt_requested():
            if (
                self.target_cache is not None
                and rospy.get_time() - self.target_cache.timestamp < CACHE_EXPIRY_SECS
            ):
                if self.target_cache.msg.distance_estimate < STOP_DISTANCE_M:
                    # Stop the rover when it is close to the ArUco tag
                    rospy.loginfo(f"Reached target {self.target_cache.msg.id}")
                    drivetrain_pub.publish(0, 0)

                    success = True
                    break
                else:
                    # Drive the rover to the target if the cache was updated recently
                    # turn = kP * self.target_cache.msg.x_offset
                    # drivetrain_pub.publish(SPEED + turn, SPEED - turn)

                    if self.target_cache.msg.x_offset > 50:
                        drivetrain_pub.publish(SPEED, 0)
                    elif self.target_cache.msg.x_offset < -50:
                        drivetrain_pub.publish(0, SPEED)
                    else:
                        drivetrain_pub.publish(SPEED, SPEED)
            else:
                # Turn the rover to look for the ArUco tag
                # drivetrain_pub.publish(SPEED, -SPEED)
                drivetrain_pub.publish(0, 0)

            rate.sleep()

        drivetrain_pub.unregister()
        sub.unregister()

        # Set result
        if success:
            self._as.set_succeeded()
        else:
            self._as.set_aborted()


if __name__ == "__main__":
    rospy.init_node("shortrange_action_server")
    ShortrangeActionServer("ShortRangeActionServer")
    rospy.spin()

## @}
# @}
