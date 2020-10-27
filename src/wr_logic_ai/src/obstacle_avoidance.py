#!/usr/bin/env python3
import math
import rospy
import time
import threading
from finder import get_navigation_angle
from angle_calculations import AngleCalculations

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
drive_pub = rospy.Publisher('/drive_cmd', DriveTrainCmd, queue_size=1)

def initialize():
    ## Subscribers
    rospy.Subscriber('/scan', LaserScan, update_navigation)
    rospy.Subscriber('/nav_data', NavigationMsg, set_target_angle)

    th = threading.Thread(target=update_target_sector)
    th.daemon = True
    th.start()


def set_target_angle(data):
    global target_angle
    imu = AngleCalculations(data.cur_lat, data.cur_long, data.tar_lat, data.tar_long)
    target_angle = imu.get_target_angle(data.heading)
    print('Target angle: ' + str(target_angle))

# TODO: consider wheter to directly call from callback
# or every 2 seconds, depends on the computational cost
def update_target_sector():
    global target_sector
    while True:
        # map angle to sector
        target_sector = target_angle / SECTOR_ANGLE

def update_navigation(data):
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
            data)

        turn_left = True if target_angle % 180 > 90 else False
        turn_left = turn_left if target_angle >= 180 else not turn_left
        if turn_left:
            print('Turn Right')
            alpha = 180
            beta = (target_angle + 90) % 360
        else:
            print('Turn Left')
            alpha = 270 - target_angle # t_a from 270-90
            beta = 180

        drive_cmd = [alpha, beta]
        drive_cmd = drive_cmd / np.linalg.norm(drive_cmd, 2)
        # drive_cmd *= 100  #LEGACY CODE, NOT IN USE THIS YEAR
        # ensures we received a valid speed factor
        speed_factor = 0.8
        speed_factor = 0 if speed_factor < 0 else speed_factor
        speed_factor = 1 if speed_factor > 1 else speed_factor
        msg = DriveTrainCmd()
        msg.left_value = drive_cmd[0] * speed_factor
        msg.right_value = drive_cmd[1] * speed_factor
        drive_pub.publish(msg)

if __name__ == '__main__':
    try:
        initialize()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        print("Stopping motors")
        msg_stop = DriveTrainCmd()
        msg_stop.left_value = 0
        msg_stop.right_value = 0
        drive_pub.publish(msg_stop)
        time.sleep(0.1)
