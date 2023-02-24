#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
import aruco_lib
# from std_msgs.msg import UInt16
# from wr_logic_ai.msg import TargetMsg
from wr_logic_ai.srv import GetTargetInfo, GetTargetInfoResponse
# from wr_logic_ai.msg import CameraInfoMsg
# from wr_logic_ai.msg import CameraInfoMsg

# target_topic = '/wr_logic_ai/shortrange_ai/vision_target_data'
# target_id_topic = '/wr_logic_ai/shortrange_ai/vision_target_id'
# camera_info_topic = '/wr_logic_ai/shortrange_ai/vision_camera_info'
# camera_info = rospy.Publisher(camera_info_topic, CameraInfoMsg, queue_size=1)

# ids = np.empty(0)
# corners = np.empty(0)
cap: cv.VideoCapture = None


def handle_get_target_info(req: GetTargetInfo):
    ret, frame = cap.read()

    if not ret:
        rospy.logerr('Failed to read frame')
    else:
        (corners, ids, _) = aruco_lib.detect_aruco(frame)
        if len(ids) != 0:
            id_list = ids.flatten().tolist()
 
            try:
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
            except:
                rospy.loginfo("Requested target not found")
    return GetTargetInfoResponse(False, [0, 0], [0, 0], 0, 0)


def main():
    global cap

    rospy.init_node('vision_target_detection')
    rospy.Service('get_target_info', GetTargetInfo, handle_get_target_info)

    stream_url = rospy.get_param('video_stream')
    rospy.loginfo("Stream: " + stream_url)

    if stream_url != None and stream_url != '':
        cap = cv.VideoCapture(stream_url)
    else:
        cap = cv.VideoCapture(0)
        cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        rospy.logerr('Failed to open camera')
        exit()

    rospy.spin()
    cap.release()


if __name__ == "__main__":
    main()
