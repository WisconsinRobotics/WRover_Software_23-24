#!/usr/bin/env python

import rospy
from wr_logic_ai.msg import NavigationMsg, TargetMsg
from std_msgs.msg import Float64
from wr_hsi_sensing.msg import CoordinateMsg

## Constants to vary
f = 100     # Hz | Rate at which pose messages are published

## INIT Objects
# Initialize node
rospy.init_node('dead_reckoning', anonymous=False)
# Create pose variable of Pose2D type
msg = NavigationMsg()
# Create array to help merge three subscriber inputs to one NavigationMsg object. 
# Index 0 is gps coordinates, index 1 is target coordinates, index 2 is heading
msg_data_input = [None, None, None]
# Create Rate object
rate = rospy.Rate(f)
# Create publisher for Pose2D data
pub_nav = rospy.Publisher('/nav_data', NavigationMsg, queue_size=1)

def init()-> None:
    rospy.Subscriber('/gps_coord_data', CoordinateMsg, update_gps_coord)
    rospy.Subscriber('/target_coord', TargetMsg, update_target_coord)
    rospy.Subscriber('/heading_data', Float64, update_heading)

def update_gps_coord(data: CoordinateMsg):
    msg_data_input[0] = data
    publish()

def update_target_coord(data : TargetMsg):
    msg_data_input[1] = data
    publish()

def update_heading(data: Float64):
    msg_data_input[2] = data
    publish()

def publish():
    msg_data_input[1] = TargetMsg()
    msg_data_input[1].target_lat = 43.0715117
    msg_data_input[1].target_long = -89.4116511

    if msg_data_input[0] is not None and msg_data_input[1] is not None and msg_data_input[2] is not None:
        msg.cur_lat = msg_data_input[0].latitude
        msg.cur_long = msg_data_input[0].longitude
        msg.tar_lat = msg_data_input[1].target_lat
        msg.tar_long = msg_data_input[1].target_long
        msg.heading = msg_data_input[2]
        pub_nav.publish(msg)

if __name__ == '__main__':
    try:
        init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
