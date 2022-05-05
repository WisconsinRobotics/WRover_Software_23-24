#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16
from wr_logic_ai.msg import TargetMsg
from wr_drive_msgs.msg import DriveTrainCmd

CAMERA_WIDTH = 1280
TARGET_IN_1M_AREA = 40000
DRIVE_SPEED = 0.1
TURN_kP = 0.0001

drivetrain_topic = '/control/drive_system/cmd'
target_topic = '/wr_logic_ai/shortrange_ai/vision_target_data'
target_id_topic = '/wr_logic_ai/shortrange_ai/vision_target_id'
drivetrain_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)
target_id_pub = rospy.Publisher(target_id_topic, UInt16, queue_size=1)
current_post = 0


def target_callback(data):
    global current_post
    # TODO handle the gate (id 4 and 5)
    if data.id == current_post:
        if at_post(data.area):
            # TODO change LED
            rospy.loginfo("Reached post, id: %d", current_post)
            drivetrain_pub.publish(0, 0)
            current_post += 1
            target_id_pub.publish(current_post)
        else:
            # TODO logic for driving through gate (ids 4 and 5)
            x_offset = CAMERA_WIDTH/2 - data.x_center
            turn_speed = TURN_kP * x_offset
            left_speed = max(DRIVE_SPEED, DRIVE_SPEED - turn_speed)
            right_speed = max(DRIVE_SPEED, DRIVE_SPEED + turn_speed)
            rospy.loginfo("left speed: %f, right speed: %f", left_speed, right_speed)
            drivetrain_pub.publish(left_speed, right_speed)
    print("I heard", str(data))


def drive(left, right):
    drivetrain_pub.publish(left, right)


def at_post(area):
    return area >= TARGET_IN_1M_AREA


def main():
    rospy.init_node('vision_navigation', anonymous=True)
    rate = rospy.Rate(10)

    target_id_pub.publish(current_post)
    rospy.Subscriber(target_topic, TargetMsg, target_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
