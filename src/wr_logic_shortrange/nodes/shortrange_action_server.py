#!/usr/bin/env python3

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

from wr_logic_shortrange.shortrange_util import TargetCache
from wr_logic_shortrange.vision_navigation import VisionNavigation
from wr_logic_shortrange.shortrange_util import ShortrangeStateEnum
from wr_logic_shortrange.msg import ShortRangeAction, ShortRangeGoal, VisionTarget
from wr_drive_msgs.msg import DriveTrainCmd

## Distance from target to stop at (in meters)
STOP_DISTANCE_M = 2
## Base speed for robot to move at
SPEED = 0.1
## Factor to for applying turn to x offset value (-1 to 1)
kP = 0.05

# Number of seconds to keep the cache
CACHE_EXPIRY_SECS = 1


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

        # Name of the drivetrain topic to publish to
        drivetrain_topic = rospy.get_param("~motor_speeds")
        # Publisher to set motor speeds
        self.drive_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)

        # Name of the VisionTarget topic to subscribe to
        vision_topic = rospy.get_param("~vision_topic")
        self.vision_sub = rospy.Subscriber(
            vision_topic, VisionTarget, self.target_callback
        )

        # TODO Reorganize to handle if we know which target is expected
        self.targets_list: List[TargetCache] = [TargetCache(0, None)] * 10

        ## The name of the action
        self._action_name = name
        ## SimpleActionServer using shortrange_callback to execute ShortrangeGoals
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ShortRangeAction,
            execute_cb=self.shortrange_callback,
            auto_start=False,
        )
        # self._as.register_preempt_callback(self.preempt_callback)
        self._as.start()

    def target_callback(self, msg: VisionTarget):
        # if msg.valid: # Update cache if the VisionTarget message is valid
        target_data = TargetCache(rospy.get_time(), msg)
        if msg.id < len(self.targets_list):
            self.targets_list[msg.id] = target_data
        self.targets_list[ShortRangeGoal.ANY] = target_data

    def shortrange_callback(self, goal: ShortRangeGoal):
        """
        Sets the shortrange state based on the ShortRangeGoal message

        @param goal (ShortRangeGoal): ShortRangeGoal message defined in action/ShortRange.action
        """
        rate = rospy.Rate(10)
        success = False
        target_id = goal.target_id

        # Loop for running ArUco tag approach
        while not rospy.is_shutdown() and not self._as.is_preempt_requested():
            target_data = self.targets_list[target_id]

            if (
                target_data.msg is not None
                and rospy.get_time() - target_data.timestamp < CACHE_EXPIRY_SECS
            ):
                if target_data.msg.distance_estimate < STOP_DISTANCE_M:
                    # Stop the rover when it is close to the ArUco tag
                    rospy.logerr(f"Shortrange reached target {self.target_cache.msg.id}")
                    self.drive_pub.publish(0, 0)

                    success = True
                    break
                else:
                    # Drive the rover to the target if the cache was updated recently
                    turn = kP * target_data.msg.x_offset
                    self.drive_pub.publish(SPEED + turn, SPEED - turn)
            else:
                # Stop the rover and wait for data
                self.drive_pub.publish(0, 0)
                # TODO add abort condition/timeout
                rospy.logerr(f"Shortrange waiting for data")

            rate.sleep()

        self.drive_pub.publish(0, 0)

        # Set result
        if success:
            self._as.set_succeeded()
        else:
            self._as.set_aborted()

    def preempt_callback(self):
        self.drive_pub(0, 0)


if __name__ == "__main__":
    rospy.init_node("shortrange_action_server")
    ShortrangeActionServer("ShortRangeActionServer")
    rospy.spin()

## @}
# @}
