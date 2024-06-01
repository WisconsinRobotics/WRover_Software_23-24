#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient, GoalStatus
from wr_logic_shortrange.msg import ShortRangeAction, ShortRangeGoal


def main():
    rospy.init_node("shortrange_action_client")
    shortrange_client = SimpleActionClient("ShortRangeActionServer", ShortRangeAction)

    shortrange_client.wait_for_server()
    shortrange_client.send_goal(ShortRangeGoal(target_id=0))
    shortrange_client.wait_for_result()
    print(shortrange_client.get_state() == GoalStatus.SUCCEEDED)


if __name__ == "__main__":
    main()
