#!/usr/bin/env python3
""" @file
@defgroup wr_logic_ai_state_machine_ai
@{
@defgroup wr_logic_ai_wait_for_user_input_py wait_for_user_input.py
@brief Service that waits for user input before continuing
@details The server will be stuck in a while loop until the user enters c. Then it will return an Emty response, allowing the code to continue

@{
"""

import rospy
import actionlib

from wr_logic_ai.msg import WaitForInputAction, WaitForInputActionGoal


class WaitForUserInputActionServer:
    def __init__(self, name) -> None:
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            WaitForInputAction,
            execute_cb=self.wait_for_user_input,
            auto_start=False,
        )
        self._as.start()

    def wait_for_user_input(self, goal: WaitForInputActionGoal):
        """
        Callback function for the action server

        Args:
            goal (actionlib goal): Empty msg
        """
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                return

            if input("Enter c to continue: ") == "c":
                self._as.set_succeeded()
                return

        self._as.set_aborted()


def main():
    """initializes rospy action server"""
    rospy.init_node("wait_for_user_input_action")
    wait_for_input_action = WaitForUserInputActionServer("WaitForUserInputActionServer")
    print("wait for user input server initialized")
    rospy.spin()


if __name__ == "__main__":
    main()

## }@
## }@
