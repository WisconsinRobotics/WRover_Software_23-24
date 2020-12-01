#!/usr/bin/env python3
import math
import rospy
import time
import threading
from finder import get_navigation_angle
from angle_calculations import AngleCalculations
import angle_to_drive_methods as angle_calc

from sensor_msgs.msg import LaserScan
from wr_logic_ai.msg import NavigationMsg
from wr_drive_msgs.msg import DriveTrainCmd

import signal
import sys
import os
import numpy as np

## Navigation parameters
HIST_TRESH = 22.5
SECTOR_COUNT = 120
SECTOR_ANGLE = 360 / SECTOR_COUNT
VISION_ANGLE = 180
# initialize target angle to move forward
target_angle = 90
target_sector = target_angle / SECTOR_COUNT

## Initialize node
rospy.init_node('nav_autonomous', anonymous=False)

## Publisher
drive_pub = rospy.Publisher('/control/drive_train_cmd', DriveTrainCmd, queue_size=1)

# Start the tasks managed to drive autonomously
def initialize() -> None:
    ## Subscribe to lidar data
    rospy.Subscriber('/scan', LaserScan, update_navigation)
    ## Subscribe to location data
    rospy.Subscriber('/nav_data', NavigationMsg, set_target_angle)

    ## Asynchronously update the target navigation
    th = threading.Thread(target=update_target_sector)
    th.daemon = True
    th.start()

## Calculate the planar target angle
def set_target_angle(data) -> None:
    global target_angle
    ## Construct the planar target angle relative to east, accounting for curvature
    imu = AngleCalculations(data.cur_lat, data.cur_long, data.tar_lat, data.tar_long)
    target_angle = imu.get_angle()
    ## Debug Out the target angle
    print('Target angle: ' + str(target_angle))

# TODO: consider wheter to directly call from callback
# or every 2 seconds, depends on the computational cost

# Update the target sector based on the target angle
def update_target_sector() -> None:
    global target_sector
    while True:
        # map angle to sector
        target_sector = target_angle / SECTOR_ANGLE

# Update the robot's navigation and drive it towards the target angle
def update_navigation(data) -> None:
    data_avg = sum(cur_range for cur_range in data.ranges) / len(data.ranges)
    # TODO: data threshold might depend of lidar model, double check
    if data_avg >= 0.5:
        # Gets best possible angle, considering obstacles
        result = get_navigation_angle(
            target_sector,
            SECTOR_COUNT,
            SECTOR_ANGLE,
            HIST_TRESH,
            VISION_ANGLE,
            data,
            smoothing_constant = rospy.get_param("smoothing_constant", 3))

        # Get the speed multiplier of the current runtime for the obstacle_avoidance
        speed_factor = rospy.get_param("speed_factor", 0.8)
        # Set the bounds of the speed multiplier
        speed_factor = 0 if speed_factor < 0 else speed_factor
        speed_factor = 1 if speed_factor > 1 else speed_factor
        
        # Get the DriveTrainCmd relating to the heading of the robot and the resulting best navigation angle
        msg = angle_calc.piecewise_linear(data.heading, result) # TODO: Double check parameters
        # Scale the resultant DriveTrainCmd by the speed multiplier
        msg.left_value *= speed_factor
        msg.right_value *= speed_factor
        # Publish the DriveTrainCmd to the topic
        drive_pub.publish(msg)

# If this file was executed...
if __name__ == '__main__':
    try:
        # Initialize the running environment for this program
        initialize()
        # Spin RosPy to the next update cycle
        rospy.spin()
    
    # Ignore ROS Interrupt Exceptions
    except rospy.ROSInterruptException:
        pass

    # If there is some other error (i.e. ROS Termination)
    finally:
        # Stop the drive motors before shutting down
        print("Stopping motors")
        msg_stop = DriveTrainCmd()
        msg_stop.left_value = 0
        msg_stop.right_value = 0
        drive_pub.publish(msg_stop)
        time.sleep(0.1)
