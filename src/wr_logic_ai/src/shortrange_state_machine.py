#!/usr/bin/env python

import rospy
import actionlib

from shortrange_util import ShortrangeAIGoal, ShortrangeAIStates, ShortrangeState
# from vision_navigation_gate import VisionNavigationGate
# from vision_navigation_post import VisionNavigationPost
from wr_logic_ai.msg import ShortrangeAction, ShortrangeGoal

class ShortrangeAIStateMachine:
    """
    Shortrange State Machine
    """

    def __init__(self, name) -> None:
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ShortrangeAction,
            execute_cb=self.shortrange_callback,
            auto_start=False
        )
        self._as.start()
        self.state = None

    def shortrange_callback(self, goal: ShortrangeGoal):
        # Set initial state based on goal
        if goal.target_type == ShortrangeAIGoal.NO_TARGET:
            self.state = ShortrangeAIStates.SUCCESS
        elif goal.target_type == ShortrangeAIGoal.ONE_TARGET:
            self.state = ShortrangeAIStates.VISION_DRIVE_POST
        elif goal.target_type == ShortrangeAIGoal.TWO_TARGETS:
            self.state = ShortrangeAIStates.VISION_DRIVE_GATE
        else:
            self.state = None
        
        if self.state == ShortrangeAIStates.SUCCESS:
            self._as.set_succeeded()
        else:
            self._as.set_aborted()


if __name__ == "__main__":
    rospy.init_node('shortrange_state_machine')
    ShortrangeAIStateMachine('shortrange_action')
    rospy.spin()
