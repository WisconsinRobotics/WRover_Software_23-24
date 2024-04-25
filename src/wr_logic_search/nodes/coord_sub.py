#!/usr/bin/env python

"""@file
@defgroup wr_logic_search
@{
@defgroup wr_logic_search_camera_sub Coordinate Subscriber
@brief 
@details 
@{
"""

import rospy
from std_msgs.msg import Float64

class CoordSub:

    start_coord = 0

    '''
    Description: Function that receives the data from the gps_coord_data topic and saves the starting latitude and longitude.

    Parameter(s):
        data - contains two float values representing the starting latitude and longitude
    '''
    def coord_callback(data):
        global start_coord
        start_coord = data.data

    '''
    Description: Function that listens to the gps_coord_data topic to get the starting latitude and longitude.
    '''
    def listener():
        rospy.init_node('coord_subscriber', anonymous=True)
        rospy.Subscriber('/gps_coord_data', Float64, CoordSub.object_detection_callback)
        rospy.spin()

    '''
    Description: Function that returns the starting latitude and longitude.

    Return(s):
        ______ - ______
    '''
    def get_coord_result():
        global start_coord
        return start_coord

if __name__ == '__main__':
    try:
        CoordSub.listener()
    except rospy.ROSInterruptException:
        pass
