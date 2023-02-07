#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan 
from wr_logic_ai.msg import NavigationMsg

from wr_drive_msgs.msg import DriveTrainCmd

import time
import math

def updateHeading(data) -> None:
    global nav
    nav.heading = (nav.heading + (data.right_value - data.left_value)*10) % 360

rospy.init_node('publish_fake_data', anonymous=False)
distanceData = rospy.Publisher('/scan', LaserScan, queue_size=10)
navigation = rospy.Publisher('/nav_data', NavigationMsg, queue_size=10)

#Testing publishers and subscribers
rospy.Subscriber('/control/drive_system/cmd', DriveTrainCmd, updateHeading)

laser = LaserScan()
#vara.intensities

inputData = []
#laser.intensities()

for x in range(360):
    distance = 5 * math.cos(x) 
    #inputData.append(str('%.5f' % distance) + " is the distance at angle " + str(x+1))
    inputData.append(distance)

#laser.angle_min = 0.
laser.angle_max = 2 * math.pi
laser.angle_increment = math.pi / 180
laser.time_increment = 0
laser.scan_time = 1
laser.range_min = 0
laser.range_max = 52
laser.ranges = inputData
laser.header.frame_id = "map"
laser.intensities = []

nav = NavigationMsg()

nav.cur_lat = 20
nav.cur_long = 10

nav.tar_lat = 30
nav.tar_long = 10
nav.heading = 270


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