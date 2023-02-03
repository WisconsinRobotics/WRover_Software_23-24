#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan 
from wr_logic_ai.msg import NavigationMsg

import time
import math

rospy.init_node('publish_fake_data')
distanceData = rospy.Publisher('/scan', LaserScan, queue_size=10)
navigation = rospy.Publisher('/nav_data', NavigationMsg, queue_size=10)


laser = LaserScan()
#vara.intensities

inputData = []
#laser.intensities()

for x in range(270):
    distance = 10
    if distance > 12:
        distance = 12
    #inputData.append(str('%.5f' % distance) + " is the distance at angle " + str(x+1))
    inputData.append(distance)
    #print(distance)

#laser.angle_min = 0.
#laser.angle_max = (2/3)*math.pi
#laser.angle_increment = math.pi/180.
#laser.time_increment = 1
laser.scan_time = 1.
laser.range_min = 0.
laser.range_max = 20.
laser.ranges = inputData
#laser.intensities = []

nav = NavigationMsg()

nav.cur_lat = 100
nav.cur_long = 100

nav.tar_lat = 100
nav.tar_long = 101
nav.heading = 0


print("sent fake nav data")


sleeper = rospy.Rate(10)
while not rospy.is_shutdown():
    
    #rospy.loginfo(nav)
    distanceData.publish(laser)
  
    navigation.publish(nav) #send nav data to subscriber in obstacle avoidance
    #print('subs')
    #rospy.spin()
    #print('spin')
    sleeper.sleep()
    



