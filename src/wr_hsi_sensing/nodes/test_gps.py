#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from wr_hsi_sensing.msg import CoordinateMsg


def gps_callback(msg):
    gps_coord_data = CoordinateMsg()
    gps_coord_data.latitude = msg.latitude
    gps_coord_data.longitude = msg.longitude

    print("latitude : {}".format(msg.latitude))
    print("longitude: {}".format(msg.longitude))

    coord_data_publisher.publish(gps_coord_data)

def gps_data_subscriber():
    rospy.init_node("gps_data_subscriber_node", anonymous = False)
    rospy.Subscriber('/gps/fix', NavSatFix, gps_callback)

    global coord_data_publisher
    coord_data_publisher = rospy.Publisher("/gps_coord_data", CoordinateMsg, queue_size=1)
    rospy.spin()

if(__name__ == "__main__"):
    gps_data_subscriber()