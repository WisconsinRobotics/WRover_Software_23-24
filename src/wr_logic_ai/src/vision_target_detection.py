#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
import aruco_lib
from wr_logic_ai.msg import TargetMsg

target_topic = '/wr_logic_ai/shortrange_ai/vision_target_data'


def process_corners(id: int, corners) -> TargetMsg:
    top_left_arr = corners[0].tolist()
    bottom_right_arr = corners[2].tolist()
    side_lengths = []
    min_x = corners[0][0]
    max_x = corners[0][0]
    for i in range(len(corners)):
        side_lengths.append(np.linalg.norm(corners[i-1] - corners[i]))
        min_x = min(min_x, corners[i][0])
        max_x = max(max_x, corners[i][0])
    x_center = (max_x + min_x) / 2
    area_estimate = max(side_lengths)**2
    return TargetMsg(id, top_left_arr, bottom_right_arr, x_center, area_estimate, True)


def main():
    pub = rospy.Publisher(target_topic, TargetMsg, queue_size=10)
    rospy.init_node('vision_target_detection')
    rate = rospy.Rate(10)

    stream_url = rospy.get_param('video_stream')
    if stream_url is not None and stream_url != '':
        cap = cv.VideoCapture(stream_url)
    else:
        cap = cv.VideoCapture(0)
        cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        rospy.logerr('Failed to open camera')
        exit()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr('Failed to read frame')
        else:
            (corners, ids, _) = aruco_lib.dettect()
            if ids is not None:
                for i, id in enumerate(ids):
                    pub.publish(process_corners(id, corners[i][0]))
            else:
                pub.publish(TargetMsg(0, [0, 0], [0, 0], 0, 0, False))
        rate.sleep()

    cap.release()


if __name__ == "__main__":
    main()
