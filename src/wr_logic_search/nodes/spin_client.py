#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient
from wr_logic_search.msg import SpinAction, SpinGoal


# Spin for 60 seconds
SPIN_TIMEOUT = 60

def main():
    rospy.init_node("spin_action_client")
    spin_client = SimpleActionClient("SpinActionServer", SpinAction)

    spin_client.wait_for_server()
    spin_client.send_goal(SpinGoal())
    spin_client.wait_for_result(rospy.Duration(SPIN_TIMEOUT))
    spin_client.cancel_goal()
    spin_client.get_state()


if __name__ == "__main__":
    main()
