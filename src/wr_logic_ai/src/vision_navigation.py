#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16
from wr_logic_ai.srv import GetTargetInfo, GetTargetInfoResponse
from wr_drive_msgs.msg import DriveTrainCmd

CAMERA_WIDTH = 1280
TARGET_IN_1M_AREA = 40000
ROTATE_SPEED = 0.1
DRIVE_SPEED = 0.1
TURN_KP = 0.0001

drivetrain_topic = '/control/drive_system/cmd'
drivetrain_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)
current_post = 0


# def target_callback(data):
#     global current_post
#     if data.id == current_post:
#         if at_post(data.area):
#             # TODO change LED
#             rospy.loginfo("Reached post, id: %d", current_post)
#             drivetrain_pub.publish(0, 0)
#             current_post += 1
#         else:
#             # TODO logic for driving through gate (ids 4 and 5)
#             x_offset = CAMERA_WIDTH/2 - data.x_center
#             turn_speed = TURN_kP * x_offset
#             left_speed = min(max(DRIVE_SPEED, DRIVE_SPEED - turn_speed), 1)
#             right_speed = min(max(DRIVE_SPEED, DRIVE_SPEED + turn_speed), 1)
#             rospy.loginfo("left speed: %f, right speed: %f", left_speed, right_speed)
#             drivetrain_pub.publish(left_speed, right_speed)
#     rospy.loginfo(str(data))


def drive(left, right):
    drivetrain_pub.publish(left, right)


def at_post(area):
    return area >= TARGET_IN_1M_AREA


def main():
    global current_post

    rospy.init_node('vision_navigation', anonymous=True)
    rate = rospy.Rate(10)

    rospy.wait_for_service('get_target_info')
    get_target_info = rospy.ServiceProxy('get_target_info', GetTargetInfo)

    while not rospy.is_shutdown():
        try:
            target_info = get_target_info(current_post)
            if target_info.is_visible:
                if at_post(target_info.area):
                    # TODO change LED
                    rospy.loginfo("Reached post, id: %d", current_post)
                    drive(0, 0)
                    current_post += 1
                else:
                    # TODO logic for driving through gate (ids 4 and 5)
                    x_offset = CAMERA_WIDTH/2 - target_info.x_center
                    turn_speed = TURN_KP * x_offset
                    left_speed = min(max(DRIVE_SPEED, DRIVE_SPEED - turn_speed), 1)
                    right_speed = min(max(DRIVE_SPEED, DRIVE_SPEED + turn_speed), 1)
                    rospy.loginfo("left speed: %f, right speed: %f",
                                  left_speed, right_speed)
                    drive(left_speed, right_speed)
            else:
                drive(-ROTATE_SPEED, ROTATE_SPEED)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

        rate.sleep()


if __name__ == "__main__":
    main()
