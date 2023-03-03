#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import UInt16
from wr_logic_ai.srv import GetTargetInfo, GetTargetInfoResponse
from wr_drive_msgs.msg import DriveTrainCmd
from geometry_msgs.msg import PoseStamped

WIDTH = 1280
TARGET_AREA = 500 ** 2
SPEED = 0.1
kP = 0.001

drivetrain_topic = '/control/drive_system/cmd'
drivetrain_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)

rviz_zero_topic = '/debug_zero'
rviz_zero = rospy.Publisher(rviz_zero_topic, PoseStamped, queue_size=10)

rviz_topic = '/debug_vision'
rviz_pub = rospy.Publisher(rviz_topic, PoseStamped, queue_size=10)

zero_msg = PoseStamped()
zero_msg.pose.position.x = 0
zero_msg.pose.position.y = 0
zero_msg.pose.position.z = 0
zero_msg.pose.orientation.x = 0
zero_msg.pose.orientation.y = 0
zero_msg.pose.orientation.z = 0
zero_msg.pose.orientation.w = 1
zero_msg.header.frame_id = 'map'
zero_msg.header.seq = 0


pose_msg = PoseStamped()
pose_msg.pose.position.x = 0
pose_msg.pose.position.y = 0
pose_msg.pose.position.z = 0
pose_msg.pose.orientation.x = 0
pose_msg.pose.orientation.y = 0
pose_msg.pose.orientation.z = 0
pose_msg.pose.orientation.w = 1
pose_msg.header.frame_id = 'map'
pose_msg.header.seq = 0



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
                # if (target_info.area < TARGET_AREA):
                turn = kP * (target_info.x_center - WIDTH / 2)
                drive(SPEED + turn, SPEED - turn)
                heading = (SPEED + turn) * -45 + (SPEED - turn) * 45
                pose_msg.pose.orientation.z = math.sin(math.radians(heading) / 2)
                pose_msg.pose.orientation.w = math.cos(math.radians(heading) / 2)
                # else:
                    # drive(0, 0)
            else:
                pose_msg.pose.orientation.w = 0
                pose_msg.pose.orientation.z = 0
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')
        rviz_zero.publish(zero_msg)
        rviz_pub.publish(pose_msg)
        rate.sleep()


if __name__ == "__main__":
    main()
