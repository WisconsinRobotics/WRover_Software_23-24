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

for x in range(120):
    distance = 100 * math.cos((x * math.pi * math.pi) / 120 / 180)
    # if x <= 10 and x >= 110:
    #     distance = 50
    inputData.append(distance)

#laser.angle_min = 0.
laser.angle_max = 2 * math.pi
laser.angle_increment = 3 * math.pi / 180
laser.time_increment = 0
laser.scan_time = 1
laser.range_min = 0
laser.range_max = 150
laser.ranges = inputData
laser.header.frame_id = "map"
laser.intensities = []

nav = NavigationMsg()

nav.cur_lat = 0
nav.cur_long = 0
nav.heading = 0

nav.tar_lat = 0
nav.tar_long = 10

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