#!/usr/bin/env python

##@defgroup wr_shortrange_ai
# @{
# @defgroup wr_shortrange_ai_vision_node Vision Target Detection Node
# @brief ROS Node for publishing vision data
# @details This ROS Node reads video input from a stream or camera.
# It reads a frame and processes it using [detect_aruco()](@ref wr_logic_ai.shortrange.aruco_lib.detect_aruco).
# Then, the data for all ArUco tags detected is published as a VisionTarget message.
# @{

import rospy
import cv2 as cv
import numpy as np
import wr_logic_ai.shortrange.aruco_lib as aruco_lib
from wr_logic_ai.msg import VisionTarget

## Width of the camera frame, in pixels
CAMERA_WIDTH = 1280
## Height of the camera frame, in pixels
CAMERA_HEIGHT = 720
## Name of the VisionTarget topic to publish to
vision_topic = rospy.get_param("~vision_topic")


def process_corners(target_id: int, corners: np.ndarray) -> VisionTarget:
    """
    Creates a VisionTarget message based on the detected ArUco tag

    @param target_id (int): ArUco tag ID
    @param corners (np.ndarray): ArUco tag corners
    @returns VisionTarget: VisionTarget message defined in msg/VisionTarget.msg
    """
    # Find the middle of the ArUco tag in the frame
    side_lengths = []
    min_x = corners[0][0]
    max_x = corners[0][0]
    for i in range(len(corners)):
        side_lengths.append(np.linalg.norm(corners[i - 1] - corners[i]))
        min_x = min(min_x, corners[i][0])
        max_x = max(max_x, corners[i][0])
    x_offset = (min_x + max_x - CAMERA_WIDTH) / 2

    # Estimate the distance of the ArUco tag in meters
    distance_estimate = aruco_lib.estimate_distance_m(corners)

    return VisionTarget(target_id, x_offset, distance_estimate, True)


def main():
    """
    @brief Vision processing node main function

    This function initializes and runs a node to read camera input and publish ArUco tag data to a topic.
    """
    pub = rospy.Publisher(vision_topic, VisionTarget, queue_size=10)
    rospy.init_node("vision_target_detection")

    rate = rospy.Rate(10)

    # Retrieve video stream from parameter server
    # If no vision stream is specified, try to use camera directly
    stream_url = rospy.get_param("~video_stream")
    if stream_url is not None and stream_url != "":
        cap = cv.VideoCapture(stream_url)
    else:
        cap = cv.VideoCapture(0)
        cap.set(cv.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

    if not cap.isOpened():
        rospy.logerr("Failed to open camera")
        exit()

    while not rospy.is_shutdown():
        # Read frame and publish detected targets
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to read frame")
        else:
            (corners, ids, _) = aruco_lib.detect_aruco(frame)
            if ids is not None:
                for i, target_id in enumerate(ids):
                    pub.publish(process_corners(target_id[0], corners[i][0]))
            else:
                # Publish even when no target is found to constantly run the navigation callback
                pub.publish(VisionTarget(0, 0, 0, False))
        rate.sleep()

    cap.release()


if __name__ == "__main__":
    main()

## @}
# @}
