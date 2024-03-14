#!/usr/bin/env python3
import os
import rospy
import rosbag
import time
from wr_drive_msgs.msg import DriveTrainCmd


dirname = os.path.dirname(__file__)
FILE_NAME = os.path.join(dirname, "drive_commands.bag")  # Relative path
topic_name = "/drive_cmd"
msg_list = []
msg_list_backwards = []
drive_pub = None


def read_data():
    rate = rospy.Rate(10)
    bag = rosbag.Bag(FILE_NAME)
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        msg_list_backwards.append(msg)
        # print(type(msg))
        # drive_pub.publish(msg)
        rate.sleep()
    for i in range(len(msg_list_backwards) - 1, 0, -1):
        drive_pub.publish(
            msg_list_backwards[i]
        )  # Error here, it expects float32 but it gets Drive
        rate.sleep()


def signal_handler(sig, frame):
    print("Script stopped by user")
    write_data()
    sys.exit(0)


def initialize():
    global drive_pub
    rospy.init_node("path_repeater", anonymous=True)

    # Publisher
    # Needs latch in order to not skip first msg
    drive_pub = rospy.Publisher(topic_name, DriveTrainCmd, queue_size=2, latch=True)
    read_data()


initialize()
