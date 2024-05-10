#!/usr/bin/env python

"""@file
@defgroup wr_logic_search
@{
@defgroup wr_logic_search_camera_sub Camera Subscriber
@brief Subscriber for the camera used in object detection
@details Subscribes the to the placeholder camera topic. It is assumed there is a separate file that does object detection. 
If the object is detected, it will return True to the Search Pattern action server. False otherwise. This Subscriber node
means search_pattern_client.py and search_pattern_server.py can be removed.
@{
"""

import rospy
from std_msgs.msg import Bool

class CameraSub:

    object_detected = False

    '''
    Description: Function that receives the data from the camera topic and saves the result of object detection.

    Parameter(s):
        data - the bool variable containing the status of object detection, True if target object detected, False otherwise
    '''
    def object_detection_callback(data):
        global object_detected
        object_detected = data.data

    '''
    Description: Function that listens to the camera topic to check if it has detected the target object.
    '''
    def listener():
        rospy.init_node('object_detection_subscriber', anonymous=True)
        rospy.Subscriber('camera/object_detection', Bool, CameraSub.object_detection_callback)
        rospy.spin()

    '''
    Description: Function that returns the status of object_detected.

    Return(s):
        object_detected - True if target object was detected, False otherwise
    '''
    def get_detection_result():
        global object_detected
        return object_detected

if __name__ == '__main__':
    try:
        CameraSub.listener()
    except rospy.ROSInterruptException:
        pass