#!/usr/bin/env python

"""@file
@defgroup wr_logic_search
@{
@defgroup wr_logic_search_camera_sub Camera Subscriber
@brief Subscriber for the camera used in object detection
@details Subscribes the to the placeholder camera topic. It is assumed there is a separate file that does object detection. 
If the object is detected, it will return True to the Search Pattern action server. False otherwise.
@{
"""

import rospy
from std_msgs.msg import Bool

class CameraSub:

    object_detected = False

    def object_detection_callback(data):
        global object_detected
        rospy.loginfo(rospy.get_caller_id() + "Received object detection event: %s", data.data)
        object_detected = data.data

    def listener():
        rospy.init_node('object_detection_subscriber', anonymous=True)
        rospy.Subscriber('camera/object_detection', Bool, CameraSub.object_detection_callback)
        rospy.spin()

    def get_detection_result():
        global object_detected
        return object_detected

if __name__ == '__main__':
    try:
        CameraSub.listener()
    except rospy.ROSInterruptException:
        pass
