#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
import aruco_lib
from std_msgs.msg import UInt16
from wr_logic_ai.msg import TargetMsg
from wr_drive_msgs.msg import DriveTrainCmd

drivetrain_topic = '/control/drive_system/cmd'
camera_width_topic = '/wr_logic_ai/shortrange_ai/vision_camera_width'
target_topic = '/wr_logic_ai/shortrange_ai/vision_target_data'
target_id_topic = '/wr_logic_ai/shortrange_ai/vision_target_id'
camera_width_pub = rospy.Publisher(drivetrain_topic, UInt16, queue_size=1)
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

    rospy.Subscriber(target_id_topic, UInt16, target_id_callback)
    
    cap = cv.VideoCapture(0)
    camera_width_pub.publish(cv.get(cv.CAP_PROP_FRAME_WIDTH))

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
               side_lengths.append(np.linalg.norm(current_target[i-1] - current_target[i]))
            area_estimate = max(side_lengths)**2
            rospy.loginfo("Target ID: %s, top left corner: %s, bottom right corner: %s, corners: %s, side_lengths: %s, area_estimate: %f", 
                str(id), str(top_left_arr), str(bottom_right_arr), str(current_target.tolist()), str(side_lengths), area_estimate)
            target_pub.publish(id, top_left_arr, bottom_right_arr, area_estimate)

        rate.sleep()

    cap.release()


if __name__ == "__main__":
    main()