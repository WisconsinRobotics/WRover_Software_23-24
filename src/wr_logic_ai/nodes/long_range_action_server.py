#!/usr/bin/env python3

import rospy
import actionlib
from wr_logic_ai.msg import LongRangeAction, LongRangeGoal
from obstacle_avoidance import update_target 

# TODO: check timeout time length validity
LONG_RANGE_TIMEOUT_TIME = rospy.Duration(5)

class LongRangeActionServer(object):
    def __init__(self, name) -> None:
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, LongRangeAction, execute_cb=self.execute_callback, auto_start=False)
        self._as.start()

    def execute_callback(self, goal: LongRangeGoal):
        start_time = rospy.get_rostime()
        while rospy.get_rostime() - start_time < LONG_RANGE_TIMEOUT_TIME:
            if update_target(goal.target_lat, goal.target_long):
                return self._as.set_succeeded()
        print("LR action server error")
        return self._as.set_aborted()

if __name__ == "__main__":
    rospy.init_node("testtest")
    server = LongRangeActionServer("LongRangeActionServer")
    rospy.spin()