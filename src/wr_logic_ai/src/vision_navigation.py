#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import UInt16
from wr_logic_ai.msg import TargetMsg
from wr_drive_msgs.msg import DriveTrainCmd
from geometry_msgs.msg import PoseStamped

# TODO : Document/rename variables, I'm not sure what all of these are for
WIDTH = 1280 # TODO : From what I can tell, this is used to figure out if we're directly facing the target.  Maybe we could publish an offset instead and avoid repeating the frame information?
TARGET_AREA = 500 ** 2
SPEED = 0.1
kP = 0.001

# TODO : Maybe use ROS params/remaps (low priority, can do later)
target_topic = '/wr_logic_ai/shortrange_ai/vision_target_data'

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


def target_callback(msg: TargetMsg):
    if msg.valid:
        turn = kP * (msg.x_center - WIDTH / 2)
        drive(SPEED + turn, SPEED - turn)
        heading = (SPEED + turn) * -45 + (SPEED - turn) * 45
        pose_msg.pose.orientation.z = math.sin(math.radians(heading) / 2)
        pose_msg.pose.orientation.w = math.cos(math.radians(heading) / 2)
        rviz_zero.publish(zero_msg)
        rviz_pub.publish(pose_msg)
    else:
        drive(0, 0)


def main():
    rospy.init_node('vision_navigation', anonymous=True)
    rospy.Subscriber(target_topic, TargetMsg, target_callback)

    rospy.spin()


if __name__ == "__main__":
    main()
