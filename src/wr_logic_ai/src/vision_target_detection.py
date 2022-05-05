#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
import aruco_lib
from std_msgs.msg import UInt16
from wr_logic_ai.msg import TargetMsg
from wr_logic_ai.msg import CameraInfoMsg
from wr_drive_msgs.msg import DriveTrainCmd

drivetrain_topic = '/control/drive_system/cmd'
camera_info_topic = '/wr_logic_ai/shortrange_ai/vision_camera_info'
target_topic = '/wr_logic_ai/shortrange_ai/vision_target_data'
target_id_topic = '/wr_logic_ai/shortrange_ai/vision_target_id'
camera_info = rospy.Publisher(camera_info_topic, CameraInfoMsg, queue_size=1)
drivetrain_pub = rospy.Publisher(drivetrain_topic, DriveTrainCmd, queue_size=1)
current_ids = np.array(0)


def target_id_callback(id):
    global current_ids
    if id not in current_ids:
        rotate(0.1)


def rotate(speed):
    drivetrain_pub.publish(speed, -speed)


def main():
    global current_ids

    target_pub = rospy.Publisher(target_topic, TargetMsg, queue_size=10)
    rospy.init_node('vision_target_detection', anonymous=True)
    rate = rospy.Rate(10)

    stream_url = rospy.get_param('~video_stream')

    rospy.Subscriber(target_id_topic, UInt16, target_id_callback)

    if stream_url != None or stream_url != '':
        cap = cv.VideoCapture(stream_url)
    else:
        cap = cv.VideoCapture(0)

    camera_info.publish(cap.get(cv.CAP_PROP_FRAME_WIDTH), cap.get(cv.CAP_PROP_FRAME_HEIGHT))

    if not cap.isOpened():
        print('Cannot open camera')
        exit()

    while cap.isOpened() and not rospy.is_shutdown():
        ret, frame = cap.read()

        if not ret:
            print('Cannot read frame')
            break

        (corners, ids, rejected) = aruco_lib.detect_markers(frame)
        current_ids = ids

        for i in range(len(corners)):
            id = ids[i].item()
            current_target = corners[i][0]
            top_left_arr = current_target[0].tolist()
            bottom_right_arr = current_target[2].tolist()
            side_lengths = []
            min_x = current_target[0][0]
            max_x = current_target[0][0]
            for i in range(len(current_target)):
               side_lengths.append(np.linalg.norm(current_target[i-1] - current_target[i]))
               min_x = min(min_x, current_target[i][0])
               max_x = max(max_x, current_target[i][0])
            x_center = (max_x - min_x) / 2
            area_estimate = max(side_lengths)**2
            rospy.loginfo("Target ID: %s, top left corner: %s, bottom right corner: %s, corners: %s, side_lengths: %s, area_estimate: %f", 
                str(id), str(top_left_arr), str(bottom_right_arr), str(current_target.tolist()), str(side_lengths), area_estimate)
            target_pub.publish(id, top_left_arr, bottom_right_arr, x_center, area_estimate)

        rate.sleep()

    cap.release()


if __name__ == "__main__":
    main()