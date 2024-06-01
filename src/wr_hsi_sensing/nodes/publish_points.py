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
#from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

import json
from pathlib import Path


def main():
    #rospy.init_node("gps_data_subscriber_node", anonymous = False)

    coord_data_publisher = rospy.Publisher("/navsat/fix", NavSatFix, queue_size=1)
    
    dirname = Path(__file__).parents[0]
    file_name = Path.joinpath(dirname, "coordinates.json")
    file = open(file_name, "r").read()
    coordinates = json.loads(file)

    rospy.loginfo(coordinates)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for coord in coordinates:
            rospy.loginfo(coord)
            coord_data_publisher.publish(NavSatFix(latitude=coord["lat"],longitude=coord["long"]))
        rate.sleep()
        
def publish_markers():
    # locations = MarkerArray()
    
    coord_data_publisher = rospy.Publisher("/marker_data", MarkerArray, queue_size=1)

    markers = []
    lat1 = 38.371768
    long1 = -110.704255
    markers.append(Marker(points=[Point(y = lat1, x = long1)]))
    
    lat2 = 38.371240
    long2 = -110.704195
    markers.append(Marker(points=[Point(y = lat2, x = long2)]))
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        coord_data_publisher.publish(MarkerArray(markers))
        
    
    
    
if(__name__ == "__main__"):
    rospy.init_node("gps_data_points_pub", anonymous = False)
    #publish_markers()
    main()
    #rospy.spin()
