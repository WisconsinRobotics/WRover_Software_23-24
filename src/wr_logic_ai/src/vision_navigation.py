#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16
from wr_logic_ai.srv import GetTargetInfo, GetTargetInfoResponse
from wr_drive_msgs.msg import DriveTrainCmd

WIDTH = 1280
TARGET_AREA = 500 ** 2
SPEED = 0.1
kP = 0.001

drivetrain_topic = '/control/drive_system/cmd'
drivetrain_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)


def drive(left: float, right: float):
    drivetrain_pub.publish(left, right)


def main():
    rospy.init_node('vision_navigation', anonymous=True)
    rate = rospy.Rate(10)

    rospy.wait_for_service('get_target_info')
    get_target_info = rospy.ServiceProxy('get_target_info', GetTargetInfo)
    current_target = 0

    while not rospy.is_shutdown():
        try:
            target_info = get_target_info(current_target)
            if target_info.is_visible:
                if (target_info.area >= TARGET_AREA):
                    turn = kP * (target_info.x_center - WIDTH / 2)
                    drive(SPEED + turn, SPEED -turn)
                else:
                    drive(0, 0)
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')
        rate.sleep()


if __name__ == "__main__":
    main()
