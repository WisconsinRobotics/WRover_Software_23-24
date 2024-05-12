#!/usr/bin/env python3    

from std_msgs.msg import Float64
import actionlib
import rospy
import testing_rviz
from wr_logic_longrange.msg import (
    InitCompassAction,
    InitCompassGoal,
)
if __name__ == '__main__':
    rospy.init_node('calculate_heading_action')
    client = actionlib.SimpleActionClient(
            "LongRangeActionServer", InitCompassAction
        )
    client.wait_for_server()
    goal = InitCompassAction()   
    client.send_goal(goal)
