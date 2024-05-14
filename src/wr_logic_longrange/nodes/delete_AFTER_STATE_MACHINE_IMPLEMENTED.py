#!/usr/bin/env python3    

from std_msgs.msg import Float64
import actionlib
import rospy
import testing_rviz
from wr_logic_ai.coordinate_manager import CoordinateManager
from wr_logic_ai.msg import NavigationState
from wr_logic_longrange.msg import (
    LongRangeAction,
    LongRangeGoal,
    LongRangeActionResult
)
if __name__ == '__main__':
    rospy.init_node('do_dishes_client')
    client = actionlib.SimpleActionClient(
            "LongRangeActionServer", LongRangeAction
        )
    client.wait_for_server()
    CoordinateManager.read_coordinates_file()
    goal = LongRangeGoal(target_lat=CoordinateManager.get_coordinate()[
                            "lat"], target_long=CoordinateManager.get_coordinate()["long"])
    rospy.loginfo(goal)
    client.send_goal(goal)
