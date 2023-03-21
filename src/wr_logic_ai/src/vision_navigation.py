#!/usr/bin/env python

import math
import rospy
from wr_logic_ai.msg import TargetMsg
from wr_drive_msgs.msg import DriveTrainCmd
from geometry_msgs.msg import PoseStamped

# TODO : Document/rename variables, I'm not sure what all of these are for
# Area in image at desired distance
TARGET_AREA = 500 ** 2
# Base speed for robot to move at
SPEED = 0.1
# Factor to for applying turn
kP = 0.001

# TODO : Maybe use ROS params/remaps (low priority, can do later)
vision_topic = rospy.get_param('vision_topic')

drivetrain_topic = '/control/drive_system/cmd'
drivetrain_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)


# Publish to topics for rviz if debug is turned on in vision_params
debug = rospy.get_param('debug', False)
if debug:
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


CACHE_EXPIRY_SECS = 1
target_cache = None


class TargetCache:
    def __init__(self, timestamp: float, msg: TargetMsg):
        self.timestamp = timestamp
        self.msg = msg


def drive(left: float, right: float):
    drivetrain_pub.publish(left, right)


def target_callback(msg: TargetMsg):
    global target_cache
    if msg.valid:
        target_cache = TargetCache(rospy.get_time(), msg)
    if target_cache is not None and rospy.get_time() - target_cache.timestamp < CACHE_EXPIRY_SECS:
        if target_cache.msg.area > TARGET_AREA:
            rospy.loginfo(f'Reached target {target_cache.msg.id}')
            drive(0, 0)
            return
        turn = kP * target_cache.msg.x_offset
        drive(SPEED + turn, SPEED - turn)
        if debug:
            heading = (SPEED + turn) * -45 + (SPEED - turn) * 45
            pose_msg.pose.orientation.z = math.sin(math.radians(heading) / 2)
            pose_msg.pose.orientation.w = math.cos(math.radians(heading) / 2)
            rviz_zero.publish(zero_msg)
            rviz_pub.publish(pose_msg)
    else:
        drive(0, 0)


def main():
    rospy.init_node('vision_navigation', anonymous=True)
    rospy.Subscriber(vision_topic, TargetMsg, target_callback)

    rospy.spin()


if __name__ == "__main__":
    main()
