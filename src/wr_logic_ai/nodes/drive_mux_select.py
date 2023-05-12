#!/usr/bin/env python3
import rospy
from wr_logic_ai.msg import NavigationState
from std_msgs.msg import String
from typing import Dict
from topic_tools.srv import MuxSelect

current = -1
class MUX:
    def __init__(self) -> None:
        self.last_message = None

    def state_call_back(self, message: NavigationState, conversion_table: Dict[int, str], mux_name : str):
        if (self.last_message_state != message.state):
            self.last_message_state = message.state
            rospy.wait_for_service(f"/{mux_name}/select")
            try:
                proxy = rospy.ServiceProxy(f"/{mux_name}/select", MuxSelect)
                proxy(conversion_table[message.state])
            except rospy.ServiceException as e:
                rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('mux_select_node')
    conversion = {
        NavigationState.NAVIGATION_STATE_LONG_RANGE: rospy.get_param("~long_range_topic_name"),
        NavigationState.NAVIGATION_STATE_SHORT_RANGE: rospy.get_param("~short_range_topic_name")
    }
    mux_instance = MUX()
    sub = rospy.Subscriber("/navigation_state", NavigationState, lambda msg: mux_instance.state_call_back(msg, conversion, rospy.get_param("~mux_name")))
    rospy.spin()
