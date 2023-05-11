#!/usr/bin/env python

from typing import Union

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from shortrange_util import ShortrangeStateEnum, ShortrangeState
from encoder_drive import EncoderDrive
from vision_navigation_gate import VisionNavigationGate
from vision_navigation_post import VisionNavigationPost
from wr_logic_ai.msg import ShortrangeAction, ShortrangeGoal


class ShortrangeStateMachine:
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
        self.state: Union[ShortrangeStateEnum, None]  = None

    def shortrange_callback(self, goal: ShortrangeGoal):
        # Set initial state based on goal
        if goal.target_type == goal.TARGET_TYPE_GPS_ONLY:
            self.state = ShortrangeStateEnum.SUCCESS
        elif goal.target_type == goal.TARGET_TYPE_SINGLE_MARKER:
            self.state = ShortrangeStateEnum.VISION_DRIVE_POST
        elif goal.target_type == goal.TARGET_TYPE_GATE:
            self.state = ShortrangeStateEnum.VISION_DRIVE_GATE
        else:
            self.state = None
        
        distance = 0
        while self.state and self.state not in ShortrangeStateEnum.TERMINATING:
            if self.state is ShortrangeStateEnum.VISION_DRIVE_POST:
                self.state, _ = VisionNavigationPost().run()
            elif self.state is ShortrangeStateEnum.VISION_DRIVE_GATE:
                self.state, distance = VisionNavigationGate().run()
            elif self.state is ShortrangeStateEnum.ENCODER_DRIVE:
                self.state, _ = EncoderDrive(distance).run()

        if self.state is ShortrangeStateEnum.SUCCESS:
            self._as.set_succeeded()
        else:
            self._as.set_aborted()


if __name__ == "__main__":
    rospy.init_node('/wr_logic_ai/shortrange_ai/state_machine')
    ShortrangeStateMachine('/wr_logic_ai/shortrange_ai/shortrange_action')
    rospy.spin()
