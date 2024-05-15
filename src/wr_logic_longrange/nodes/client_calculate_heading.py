#!/usr/bin/env python3

from std_msgs.msg import Float64
import actionlib
import rospy
import testing_rviz
from wr_logic_longrange.msg import (
    InitCompassAction,
    InitCompassGoal,
)

# TODO remove file after integration with state machine
if __name__ == "__main__":
    rospy.init_node("client_calculate_heading_action")
    client = actionlib.SimpleActionClient("InitCompass", InitCompassAction)
    client.wait_for_server()
    rospy.loginfo("Sending GOAL")
    goal = InitCompassAction()
    client.send_goal(goal)
