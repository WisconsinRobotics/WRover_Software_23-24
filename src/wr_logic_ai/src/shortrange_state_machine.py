#!/usr/bin/env python

from enum import Enum

import rospy

from shortrange_util import ShortrangeAIStates, ShortrangeState


class ShortrangeAIStateMachine:
    def __init__(self, state: ShortrangeAIStates) -> None:
        self.state = state
        rospy.init_node('shortrange_state_machine')

    def runState(self, state: ShortrangeState):
        self.state = state.run()


if __name__ == "__main__":
    pass