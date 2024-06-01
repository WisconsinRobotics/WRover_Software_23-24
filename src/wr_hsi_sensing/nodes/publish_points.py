#!/usr/bin/env python3

## @file
# @brief Node responsible for mapping Ardusimple GPS data to CoordinateMsg
# @ingroup wr_hsi_sensing
# @defgroup wr_hsi_sensing_gps GPS
# @details Subscribes to the /gps/fix topic and publishes the latitude and longitude data to the /gps_coord_data topic
# @{

import rospy
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point



def main():
    rospy.init_node("gps_data_subscriber_node", anonymous = False)

    coord_data_publisher = rospy.Publisher("/navsat/fix", NavSatFix, queue_size=1)
    
    lat1 = 38.371768
    long1 = -110.704255
    coord_data_publisher.publish(NavSatFix(latitde=lat1,longitude=long1))
    
    lat2 = 38.371240
    long2 = -110.704195
    coord_data_publisher.publish(NavSatFix(latitde=lat2,longitude=long2))
        
def publish_markers():
    # locations = MarkerArray()
    
    coord_data_publisher = rospy.Publisher("/marker_data", MarkerArray, queue_size=1)

    markers = []
    lat1 = 38.371768
    long1 = -110.704255
    markers.add(Marker(points=[Point(y = lat1, x = long1)]))
    
    lat2 = 38.371240
    long2 = -110.704195
    markers.add(Marker(points=[Point(y = lat2, x = long2)]))
    
    coord_data_publisher.publish(MarkerArray(markers))
    
    
    
if(__name__ == "__main__"):
    rospy.init_node("gps_data_points_pub", anonymous = False)
    publish_markers()
    rospy.spin()