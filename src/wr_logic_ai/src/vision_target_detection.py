#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
import aruco_lib
from std_msgs.msg import UInt8
from wr_logic_ai.msg import TargetMsg
from wr_drive_msgs.msg import DriveTrainCmd

drivetrain_topic = '/control/drive_system/cmd'
target_topic = '/wr_logic_ai/shortrange_ai/vision_target_data'
target_id_topic = '/wr_logic_ai/shortrange_ai/vision_target_id'
drivetrain_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)
current_ids = np.array(0)


def target_id_callback(id):
    if id not in current_ids:
        rotate(0.1)


def rotate(speed):
    drivetrain_pub.publish(speed, -speed)


def main():
    target_pub = rospy.Publisher(target_topic, TargetMsg, queue_size=10)
    rospy.init_node('vision_target_detection', anonymous=True)
    rate = rospy.Rate(10)

    rospy.Subscriber(target_id_topic, UInt8, target_id_callback)
    
    cap = cv.VideoCapture(0)

    if not cap.isOpened():
        print('Cannot open camera')
        exit()

    while cap.isOpened() and not rospy.is_shutdown():
        ret, frame = cap.read()

        if not ret:
            print('Cannot read frame')
            break

        (corners, ids, rejected) = aruco_lib.detect_markers(frame)

        for i in range(len(corners)):
            id = ids[i].item()
            current_target = corners[i][0]
            top_left_arr = current_target[0].tolist()
            bottom_right_arr = current_target[2].tolist()
            side_lengths = []
            for i in range(len(current_target)):
                side_lengths.append(np.linalg.norm(current_target[i] - current_target[i+1]))
            area_estimate = max(side_lengths)**2
            rospy.loginfo(id, top_left_arr, bottom_right_arr, area_estimate)
            target_pub.publish(id, top_left_arr, bottom_right_arr, area_estimate)

        rate.sleep()

    cap.release()


if __name__ == "__main__":
    main()