#!/usr/bin/env python

import json
import rospy
import os
from threading import Thread
from BNO055 import BNO055
from wr_logic_ai.msg import NavigationMsg
import gps

## Constants to vary
f = 100     # Hz | Rate at which pose messages are published

## INIT Objects
# Initialize node
rospy.init_node('dead_reckoning', anonymous=False)
# Create pose variable of Pose2D type
msg = NavigationMsg()
# Create Rate object
rate = rospy.Rate(f)
# Create publisher for Pose2D data
pub_nav = rospy.Publisher('/nav_data', NavigationMsg, queue_size=1)

# Create IMU Sensor object
sensor = BNO055()
if not sensor.begin(mode=BNO055.OPERATION_MODE_IMUPLUS):
    raise RuntimeError("IMU Failed to initialize")
sensor.setExternalCrystalUse(True)
# Dump some initial IMU readings
imuInitCounter = 0
while((sensor.getVector(BNO055.VECTOR_EULER)[2] == 0.0)and
        (imuInitCounter > 100)):
    print(sensor.getVector(BNO055.VECTOR_EULER))
    rate.sleep()
    imuInitCounter+=1

# TODO : New GPS is I2C, need to double check that this implementation is ok.
# Listen on port 2947 of localhost for gps
session = gps.gps("localhost", "2947")
session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
# Create array for storing previous location
previous_coordinates = [0, 0]
# Space for current coordinates
current_coordinates = {'lat': 0, 'long': 0}

## User should provide the current desired goal coordinates
def get_target_coordinates():
    dirname = os.path.dirname(__file__)
    file_name = os.path.join(dirname, 'coordinates.json')
    coordinates = open(file_name, 'r').read()
    c_json = json.loads(coordinates)
    return c_json

## Compute x and y position from GPS (returns a dictionary [x,y])
def get_coordinates():
    global current_coordinates
    # Create empty array
    global previous_coordinates
    while True:
        coordinate_array = [0, 0]
        report = session.next()
        if report['class'] == 'TPV':
            if hasattr(report, 'lat'): ## ------------- This lines have to be separeted? Or can they be like below?
                if hasattr(report, 'lon'): ## --------- if hasattr(report, 'lat') and hasattr(report, 'lon'):
                    coordinate_array[0] = report.lat
                    coordinate_array[1] = report.lon
                    # update previous_coordinates with new coordinates if they are not 0
                    if (not(report.lat == 0)): ## ----- Probably would be better to write report.lat != 0 for readability
                        previous_coordinates[0] = coordinate_array[0]
                        previous_coordinates[1] = coordinate_array[1]
        # returns previous coordinates if current coordinates are retrieved as 0
        if coordinate_array[0] == 0:
            current_coordinates = {'lat': previous_coordinates[0], 'long': previous_coordinates[1]}
        # returns current coordinates if they are not retrieved as 0
#        current_coordinates = {'lat': coordinate_array[0], 'long': coordinate_array[1]}
        current_coordinates = {'lat':43.0727, 'long': -89.412}
        print("updated coordinates: {}".format(current_coordinates))

## Retrieve heading from IMU (Returns a float angle)
def get_heading():
    # TODO: get actual obset from an absolute value ---- Build calibration method for startup.
    offset_from_east = 90 
    # imu returns clockwise values, we need it counterclockwise
    relative_angle = 360 - sensor.getVector(BNO055.VECTOR_EULER)[0]
    return (offset_from_east + relative_angle) % 360

## Publishes pose data continuously when called
def talker():
    global current_coordinates
    while not rospy.is_shutdown():
        ## Build nav_data msg to send
        #Target coords
        target_coords_dict = get_target_coordinates()
        msg.tar_lat = target_coords_dict['lat']
        msg.tar_long = target_coords_dict['long']
        #Current coords
        msg.cur_lat = current_coordinates['lat']
        msg.cur_long = current_coordinates['long']
        #Heading
        msg.heading = get_heading()

        # Publish pose at f rate
        pub_nav.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Make separate thread for gps
        thread_gps = Thread(target=get_coordinates)
        thread_gps.daemon=True
        thread_gps.start()
        talker()
    except rospy.ROSInterruptException:
        pass
