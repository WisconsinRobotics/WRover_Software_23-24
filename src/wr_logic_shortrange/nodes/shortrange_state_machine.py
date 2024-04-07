#!/usr/bin/env python

##@defgroup wr_shortrange_ai
# @{
# @defgroup wr_shortrange_ai_state_machine Shortrange State Machine
# @brief Node for the shortrange state machine
# @details The shortrange state machine implements the state machine depicted below.
#
# The state machine runs when the ActionServer callback, ShortrangeStateMachine.shortrange_callback(), is executed.
#
# Currently, the only state is [VisionNavigation](@ref wr_logic_ai.shortrange.vision_navigation.VisionNavigation).
# This state drives the rover to an ArUco tag.
#
# ![](ShortrangeStateMachine.png)
# @{

from typing import *

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from wr_logic_shortrange.shortrange_util import ShortrangeStateEnum, ShortrangeState
from wr_logic_shortrange.vision_navigation import VisionNavigation
from wr_logic_shortrange.shortrange_util import ShortrangeStateEnum
from wr_logic_shortrange.msg import ShortRangeAction, ShortRangeGoal


class ShortrangeStateMachine:
    """
    @brief Shortrange state machine class

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
        ## The current state of the state machine
        self.state: ShortrangeStateEnum = None

    def shortrange_callback(self, goal: ShortRangeGoal):
        """
        Sets the shortrange state based on the ShortRangeGoal message

        @param goal (ShortRangeGoal): ShortRangeGoal message defined in action/ShortRange.action
        """

        # Set initial state based on goal
        if goal.target_type == goal.TARGET_TYPE_GPS_ONLY:
            self.state = ShortrangeStateEnum.SUCCESS
        elif goal.target_type == goal.TARGET_TYPE_VISION:
            self.state = ShortrangeStateEnum.VISION_DRIVE
        else:
            self.state = None

        # Run states while the current state is not a terminating state
        while (
            not rospy.is_shutdown()
            and self.state
            and self.state not in ShortrangeStateEnum.TERMINATING
        ):
            if self.state is ShortrangeStateEnum.VISION_DRIVE:
                self.state = VisionNavigation().run()

        # Set result of
        if self.state is ShortrangeStateEnum.SUCCESS:
            self._as.set_succeeded()
        else:
            self._as.set_aborted()


if __name__ == "__main__":
    rospy.init_node("shortrange_state_machine")
    ShortrangeStateMachine("ShortRangeActionServer")
    rospy.spin()

## @}
# @}
