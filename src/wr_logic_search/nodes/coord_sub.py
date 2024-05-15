#!/usr/bin/env python

"""@file
@defgroup wr_logic_search
@{
@defgroup wr_logic_search_camera_sub Coordinate Subscriber
@brief Subscriber for the coordinate topic
@details Subscribes to the /gps_coord_data topic. Listens to the topic until it 
receives the starting latitude and longitude. Theses values will be saved for the 
rover to eventually travel to.
@{
"""

import rospy
from wr_hsi_sensing.msg import CoordinateMsg

start_lat = 0
start_long = 0

'''
Description: Initializes the Subscriber node that listens to the 
    /gps_coord_data topic to get the starting latitude and longitude.
'''
def initialize():
    rospy.init_node('coord_subscriber', anonymous=True)
    rospy.Subscriber("/gps_coord_data", CoordinateMsg, coord_callback)

'''
Description: Receives the data from the gps_coord_data topic and 
    saves the starting latitude and longitude.

Parameter(s):
    msg - contains two float values representing the starting latitude and 
        longitude
'''
def coord_callback(msg: CoordinateMsg):
    global start_lat
    global start_long
    start_lat = msg.latitude
    start_long = msg.longitude

'''
Description: Function that returns the starting latitude and longitude.

Return(s):
    start_lat - the starting latitude the rover will travel to 
    start_long - the starting longitude the rover will travel to 
'''
def get_coord_result():
    global start_lat
    global start_long
    return start_lat, start_long

if __name__ == '__main__':
    try:
        initialize()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
