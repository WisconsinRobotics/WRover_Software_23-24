#!/usr/bin/env python3

"""@file
@defgroup wr_logic_search
@{
@defgroup wr_logic_search_spin Spin Action Server
@brief Subscriber for the coordinate topic
@details Spins the rover and searches for an object
@{
"""

import rospy
import actionlib

from std_msgs.msg import Float64

from wr_drive_msgs.msg import DriveTrainCmd
from wr_logic_search.msg import SpinAction, SpinGoal
from wr_logic_shortrange.msg import VisionTarget


class SpinActionServer:
    TURN_SPEED = 0.12

    def __init__(self, name) -> None:

        self.current_heading = 0
        self.heading_time = 0

        heading_topic = rospy.get_param("~heading_topic")
        self.heading_sub = rospy.Subscriber(
            heading_topic, Float64, self.heading_callback
        )

        self.vision_target = 0
        self.vision_time = 0

        vision_topic = rospy.get_param("~vision_topic")
        self.vision_sub = rospy.Subscriber(
            vision_topic, VisionTarget, self.vision_callback
        )

        # Name of the drivetrain topic to publish to
        drivetrain_topic = rospy.get_param("~motor_speeds")
        # Publisher to set motor speeds
        self.drive_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            SpinAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self._as.start()

    def heading_callback(self, msg: Float64):
        self.current_heading = msg.data
        self.heading_time = rospy.get_time()

    def vision_callback(self, msg: VisionTarget):
        self.vision_target = msg.id
        self.vision_time = rospy.get_time()

    def execute_callback(self, goal: SpinGoal):
        success = False
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and self.heading_time == 0:
            rate.sleep()

        initial_heading = self.current_heading
        start_time = self.heading_time
        while not rospy.is_shutdown() and not self._as.is_preempt_requested():
            if self.vision_time != 0:
                # Check that target is visible for a certain amount of time
                self.last_time = self.vision_time
                self.drive_pub.publish(0, 0)
                for _ in range(5):
                    rospy.sleep(0.5)
                    if self.last_time == self.vision_time:
                        self.vision_time = 0
                        break
                else:
                    # If loop exited normally, we have a good view of the vision target
                    success = True
                    break
            elif (
                abs(self.current_heading - initial_heading) < 5
                # TODO don't use a magical constant here
                and self.heading_time - start_time > 10
            ):
                break

            self.drive_pub.publish(-self.TURN_SPEED, self.TURN_SPEED)
            rate.sleep()

        self.drive_pub.publish(0, 0)

        if success:
            self._as.set_succeeded()
        else:
            self._as.set_aborted()


if __name__ == "__main__":
    try:
        rospy.init_node("spin_action_server")
        server = SpinActionServer("SpinActionServer")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

## @}
## @}
