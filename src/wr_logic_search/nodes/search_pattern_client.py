#!/usr/bin/env python3

"""@file
@defgroup wr_logic_search
@{
@defgroup wr_logic_search_search_pattern_client Search Pattern Client
@brief The client for the camera that scans the rover's surroundings for the target object.
@details This client is currently waiting for the response from the server and sending the 
result (boolean), which seems a little redundant. This might be changed or completely 
removed in the future.
@{
"""

import rospy
from actionlib import SimpleActionClient

from wr_logic_longrange.msg import InitCompassAction, InitCompassGoal
from wr_logic_search.msg import SearchStateAction, SearchStateGoal


def main():
    rospy.init_node("search_pattern_client")
    compass_client = SimpleActionClient("InitCompassActionServer", InitCompassAction)
    spin_client = SimpleActionClient("SearchActionServer", SearchStateAction)

    # Input test coordinates
    test_lat, test_long = 43.0724525, -89.4106448
    test_target = SearchStateGoal.ANY

    compass_client.wait_for_server()
    compass_client.send_goal(InitCompassGoal())
    compass_client.wait_for_result()

    spin_client.wait_for_server()
    spin_client.send_goal(
        SearchStateGoal(
            initial_lat=test_lat, initial_long=test_long, target_id=test_target
        )
    )
    spin_client.wait_for_result()
    spin_client.get_state()
    rospy.init_node("search_pattern_client")


if __name__ == "__main__":
    main()

## @}
## @}
