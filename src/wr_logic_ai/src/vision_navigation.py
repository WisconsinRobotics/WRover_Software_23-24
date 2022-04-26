#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8
from wr_logic_ai.msg import TargetMsg
from wr_drive_msgs.msg import DriveTrainCmd

drivetrain_topic = '/control/drive_system/cmd'
target_topic = '/wr_logic_ai/shortrange_ai/vision_target_data'
target_id_topic = '/wr_logic_ai/shortrange_ai/vision_target_id'
drivetrain_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)
target_id_pub = rospy.Publisher(target_id_topic, UInt8, queue_size=1)
current_post = 1


def target_callback(data):
    # TODO handle the gate (id 4 and 5)
    if data.id == current_post:
        if at_post():
            # TODO change LED
            drivetrain_pub.publish(0, 0)
            current_post += 1
            target_id_pub.publish(current_post)
        else:
            # TODO logic for driving to target
            drivetrain_pub.publish(0.1, 0.1)
    print("I heard", str(data))


def drive(left, right):
    drivetrain_pub.publish(left, right)


def at_post():
    # TODO tune using area of target to estimate distance from post
    return False


def main():
    rospy.init_node('vision_navigation', anonymous=True)
    rate = rospy.Rate(10)

    target_id_pub.publish(current_post)
    rospy.Subscriber(target_topic, TargetMsg, target_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
