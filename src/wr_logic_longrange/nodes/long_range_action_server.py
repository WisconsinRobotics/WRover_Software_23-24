#!/usr/bin/env python3

"""@file
@defgroup wr_logic_longrange
@{
@defgroup wr_logic_longrange_action_server Long Range Action Server
@brief Action server for long range navigation
@details This action server is used in conjunction with the state machine to drive the long range 
navigation logics. The action server determines if a navigation is successful (when it reaches the 
destination coordinate), or if it failed (when a timeout is triggered), and triggers a state machine 
event accordingly. 
@{
"""

import rospy
import actionlib
from wr_logic_longrange.msg import LongRangeAction, LongRangeGoal
import obstacle_avoidance

# Temporary:
from std_msgs.msg import Float64
import testing_rviz
from wr_logic_longrange.msg import (
    InitCompassAction,
    InitCompassGoal,
)
from wr_drive_msgs.msg import DriveTrainCmd
from sensor_msgs.msg import LaserScan


# TODO: check timeout time length validity
## Timeout time for when we declare a long range navigation as failed
LONG_RANGE_TIMEOUT_TIME = rospy.Duration(1000)


class LongRangeActionServer(object):
    """
    Class for long range navigation's action server

    @param object (_type_): Unused
    """

    def __init__(self, name) -> None:
        rospy.loginfo("initing long range action server")
        self._action_name = name
        # Publisher
        self.drive_pub = rospy.Publisher(
            rospy.get_param("~motor_speeds"), DriveTrainCmd, queue_size=10
        )
        obstacle_avoidance.initialize()
        
        # Subscribe to lidar data
        rospy.wait_for_message("/scan", LaserScan, timeout=None)


        self._as = actionlib.SimpleActionServer(
            self._action_name,
            LongRangeAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self._as.start()

    def stop_motors(self):
        stop_msg = DriveTrainCmd()
        stop_msg.left_value = 0
        stop_msg.right_value = 0
        self.drive_pub.publish(stop_msg)

    def execute_callback(self, goal: LongRangeGoal):
        """
        Executes the long range obstacle avoidance code, and triggers the corresponding state machine event
        depending on the result of the navigation

        @param goal (LongRangeGoal): Goal for the navigation segment, which contains the GPS coordinates
        of the target
        """
        rate = rospy.Rate(10)
        start_time = rospy.get_rostime()
        while (
            rospy.get_rostime() - start_time < LONG_RANGE_TIMEOUT_TIME
            and not rospy.is_shutdown()
        ):
            rate.sleep()
            if obstacle_avoidance.update_target(goal.target_lat, goal.target_long):
                rospy.loginfo("SUCCESS LONG RANGE ACTION SERVER")
                # Stop motors
                self.stop_motors()
                return self._as.set_succeeded()
            else:
                # set motor to drive to target
                msg = obstacle_avoidance.get_drive_power()
                self.drive_pub.publish(msg)

        self.stop_motors()
        return self._as.set_aborted()


if __name__ == "__main__":
    rospy.init_node("long_range_action_server")
    server = LongRangeActionServer("LongRangeActionServer")
    rospy.spin()
