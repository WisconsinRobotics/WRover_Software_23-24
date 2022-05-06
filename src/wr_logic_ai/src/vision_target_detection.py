#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
import aruco_lib
from std_msgs.msg import UInt16
# from wr_logic_ai.msg import TargetMsg
from wr_logic_ai.srv import GetTargetInfo, GetTargetInfoResponse
from wr_logic_ai.msg import CameraInfoMsg
from wr_logic_ai.msg import CameraInfoMsg

# target_topic = '/wr_logic_ai/shortrange_ai/vision_target_data'
# target_id_topic = '/wr_logic_ai/shortrange_ai/vision_target_id'
camera_info_topic = '/wr_logic_ai/shortrange_ai/vision_camera_info'
camera_info = rospy.Publisher(camera_info_topic, CameraInfoMsg, queue_size=1)

ids = np.empty(0)
corners = np.empty(0)


def handle_get_target_info(req: GetTargetInfo):
    if len(corners) == 0:
        return GetTargetInfoResponse(False, [0, 0], [0, 0], 0, 0)

    id_list = ids.flatten().tolist()
    if req.id not in id_list:
        return GetTargetInfoResponse(False, [0, 0], [0, 0], 0, 0)

    id_ind = id_list.index(req.id)
    target = corners[id_ind][0]
    top_left_arr = target[0].tolist()
    bottom_right_arr = target[2].tolist()
    side_lengths = []
    min_x = target[0][0]
    max_x = target[0][0]
    for i in range(len(target)):
        side_lengths.append(np.linalg.norm(target[i-1] - target[i]))
        min_x = min(min_x, target[i][0])
        max_x = max(max_x, target[i][0])
    x_center = (max_x - min_x) / 2
    area_estimate = max(side_lengths)**2

    rospy.loginfo("Target ID: %s, top left corner: %s, bottom right corner: %s, corners: %s, side_lengths: %s, area_estimate: %f",
                  str(id), str(top_left_arr), str(bottom_right_arr), str(target.tolist()), str(side_lengths), area_estimate)
    return GetTargetInfoResponse(True, top_left_arr, bottom_right_arr, x_center, area_estimate)


def main():
    global ids, corners

    rospy.init_node('vision_target_detection')
    rospy.Service('get_target_info', GetTargetInfo, handle_get_target_info)

    rate = rospy.Rate(10)

    stream_url = rospy.get_param('video_stream')
    rospy.loginfo("Stream: " + stream_url)

    if stream_url != None and stream_url != '':
        cap = cv.VideoCapture(stream_url)
    else:
        cap = cv.VideoCapture(0)

    camera_info.publish(cap.get(cv.CAP_PROP_FRAME_WIDTH),
                        cap.get(cv.CAP_PROP_FRAME_HEIGHT))

    if not cap.isOpened():
        rospy.logerr('Cannot open camera')
        exit()

    while cap.isOpened() and not rospy.is_shutdown():
        ret, frame = cap.read()

        if not ret:
            rospy.logdebug('Cannot read frame')
            break

        (corners, ids, rejected) = aruco_lib.detect_markers(frame)

    #     for i in range(len(corners)):
    #         id = ids[i].item()
    #         current_target = corners[i][0]
    #         top_left_arr = current_target[0].tolist()
    #         bottom_right_arr = current_target[2].tolist()
    #         side_lengths = []
    #         min_x = current_target[0][0]
    #         max_x = current_target[0][0]
    #         for i in range(len(current_target)):
    #            side_lengths.append(np.linalg.norm(current_target[i-1] - current_target[i]))
    #            min_x = min(min_x, current_target[i][0])
    #            max_x = max(max_x, current_target[i][0])
    #         x_center = (max_x - min_x) / 2
    #         area_estimate = max(side_lengths)**2
    #         rospy.loginfo("Target ID: %s, top left corner: %s, bottom right corner: %s, corners: %s, side_lengths: %s, area_estimate: %f",
    #             str(id), str(top_left_arr), str(bottom_right_arr), str(current_target.tolist()), str(side_lengths), area_estimate)
    #         target_pub.publish(id, top_left_arr, bottom_right_arr, x_center, area_estimate)

        rate.sleep()

    cap.release()


if __name__ == "__main__":
    main()
