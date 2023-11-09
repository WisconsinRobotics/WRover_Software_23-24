#!/usr/bin/env python
""" @file
@defgroup wr_logic_ai_state_machine_ai
@{
@defgroup wr_logic_ai_wait_for_user_input_py wait_for_user_input.py
@brief Service that waits for user input before continuing
@details The server will be stuck in a while loop until the user enters c. Then it will return an Emty response, allowing the code to continue

@{
"""


import rospy
from std_srvs.srv import Empty, EmptyResponse

# TODO(@bennowotny): Should be an action server due to time taken, preemptibility


def wait_for_user_input(req):
    """callback function for the service

    Args:
        req (service message): Empty

    Returns:
        EmmptyResponse: None
    """
    print("Service called")
    while True:
        if input("Enter c to continue: ") == "c":
            return EmptyResponse()


def wait_for_user_input_server():
    """initializes rospy service"""
    rospy.init_node("wait_for_user_input_server")
    s = rospy.Service("wait_for_user_input_service", Empty, wait_for_user_input)
    print("wait for user input server initialized")
    rospy.spin()


# TODO: Figure what to do with this
def shutdown_server():
    rospy.signal_shutdown("Ended user signal")


if __name__ == "__main__":
    wait_for_user_input_server()

## }@
## }@
