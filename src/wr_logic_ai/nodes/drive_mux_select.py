#!/usr/bin/env python3

"""@file
@defgroup wr_logic_ai_mux_selector Mux_Selector
@ingroup wr_logic_ai
@brief Node for switching between long-range navigation output and short-range navigation output to 
rover motor power inputs
@details In the world of computer engineering, MUX is short for multiplexer. A multiplexer is a logic 
circuit that takes in multiple input signals, chooses one among them, and outputs that signal.

Here, the mux select node performs a similar function by choosing a motor power output between 
long-range logics and short-range logics and forwards it to the actual motor power topic. The 
switching behavior is controlled by which state the state machine is currently in. The purpose of 
doing this extra step is to add a layer of abstraction between higher level logics and low level 
controls. Both long range and short range believes that they have complete control over the rover, 
thereby simplifying their implementations. 
@{
"""

import rospy
from wr_logic_ai.msg import NavigationState
from std_msgs.msg import String
from typing import Dict
from topic_tools.srv import MuxSelect


class MUX:
    """
    This class implements the MUX selector
    """

    def __init__(self) -> None:
        ## Saves the current mux output source
        self.last_message_state = None

    def state_call_back(
        self, message: NavigationState, conversion_table: Dict[int, str], mux_name: str
    ):
        """
        Switches the output source of the mux according to the given NavigationState message

        @param message (NavigationState): Output source for the mux (either long-range or short-range)
        @param conversion_table (Dict[int, str]): Converts the message content to either long-range or
        @param short-range's output topic name
        @param mux_name (str): This mux node's output topic name
        """
        if self.last_message_state != message.state:
            self.last_message_state = message.state
            rospy.wait_for_service(f"/{mux_name}/select")
            try:
                proxy = rospy.ServiceProxy(f"/{mux_name}/select", MuxSelect)
                proxy(conversion_table[message.state])
            except rospy.ServiceException as e:
                rospy.logerr(e)


if __name__ == "__main__":
    """
    Initializes the MUX object and sets up the subscriber callback function
    """

    rospy.init_node("mux_select_node")
    conversion = {
        NavigationState.NAVIGATION_STATE_LONG_RANGE: rospy.get_param(
            "~long_range_topic_name"
        ),
        NavigationState.NAVIGATION_STATE_SHORT_RANGE: rospy.get_param(
            "~short_range_topic_name"
        ),
    }
    mux_instance = MUX()
    sub = rospy.Subscriber(
        "/navigation_state",
        NavigationState,
        lambda msg: mux_instance.state_call_back(
            msg, conversion, rospy.get_param("~mux_name")
        ),
    )
    rospy.spin()

## }@
