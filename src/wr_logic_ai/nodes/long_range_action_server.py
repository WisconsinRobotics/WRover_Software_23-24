#!/usr/bin/env python3

import rospy
import actionlib
from wr_logic_ai.msg import LongRangeAction, LongRangeResult
from obstacle_avoidance import update_target

# TODO: check timeout time length validity
LONG_RANGE_TIMEOUT_TIME = rospy.Duration(500)

class LongRangeActionServer(object):
    _result = LongRangeResult()

    def __init__(self, name) -> None:
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, LongRangeAction, execute_cb=self.execute_callback, auto_start=False)
        self._as.start()

    def execute_callback(self, goal):
        start_time = rospy.get_rostime()
        while rospy.get_rostime() - start_time < LONG_RANGE_TIMEOUT_TIME:
            if update_target(goal.target_lat, goal.target_long):
                self._result.nav_state_type = LongRangeResult.NAV_STATE_TYPE_COMPLETE
                self._as.set_succeeded(self._result)
        self._result.nav_state_type = LongRangeResult.NAV_STATE_TYPE_ERROR
        self._as.set_succeeded(self._result)

if __name__ == "__main__":
    rospy.init_node("LongRangeActionServer")
    server = LongRangeAction(rospy.get_name())
    rospy.spin()