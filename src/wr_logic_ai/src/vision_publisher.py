#!/usr/bin/env python

import rospy
import cv2 as cv
import aruco_lib
from wr_logic_ai.msg import TargetMsg

topic_name = '/wr_logic_ai/shortrange_ai/vision'

def main():
    pub = rospy.Publisher(topic_name, TargetMsg, queue_size=10)
    rospy.init_node('vision', anonymous=True)
    rate = rospy.Rate(10)
    
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
            top_left = corners[i][0][0].tolist()
            bottom_right = corners[i][0][2].tolist()
            id = ids[i].item()
            pub.publish(top_left, bottom_right, id)

        rate.sleep()

    cap.release()


if __name__ == "__main__":
    main()