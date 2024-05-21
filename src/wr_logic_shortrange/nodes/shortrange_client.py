#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient
from wr_logic_shortrange.msg import ShortRangeAction, ShortRangeGoal

def main():
    rospy.init_node("shortrange_action_client")
    shortrange_client = SimpleActionClient("ShortRangeActionServer", ShortRangeAction)

    shortrange_client.wait_for_server()
    shortrange_client.send_goal(ShortRangeGoal(target_type=1))
    shortrange_client.wait_for_result()
    shortrange_client.get_state()

if __name__ == '__main__':
    main()
