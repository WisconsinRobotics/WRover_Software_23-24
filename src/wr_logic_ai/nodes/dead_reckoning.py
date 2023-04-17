#!/usr/bin/env python

import json
import rospy
import os
import time
from threading import Thread
from BNO055 import BNO055
from wr_logic_ai.msg import NavigationMsg
from adafruit_gps import I2C,GPS_GtopI2C as GPS
from wr_logic_ai.msg import TargetMsg

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
#Target Coordinates sent from state machine
target_coords = []


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

# Space for current coordinates
current_coordinates = {'lat': 0, 'long': 0}

def init()-> None:
    rospy.Subscriber('/target_coord', TargetMsg, talker)
    
## User should provide the current desired goal coordinates
## Not used Anymore and moved to state machine
def get_target_coordinates():
    dirname = os.path.dirname(__file__)
    file_name = os.path.join(dirname, 'coordinates.json')
    coordinates = open(file_name, 'r').read()
    c_json = json.loads(coordinates)
    return c_json

## Compute x and y position from GPS (returns a dictionary [x,y])
def get_coordinates(gps : GPS):
    global current_coordinates
    while True:
        gps.update()
        if(gps.latitude is not None and gps.longitude is not None):
            current_coordinates['lat'] = gps.latitude
            current_coordinates['long'] = gps.longitude
        print("updated coordinates: {}".format(current_coordinates))
        time.sleep(0.1)

## Retrieve heading from IMU (Returns a float angle)
def get_heading():
    # TODO: get actual obset from an absolute value ---- Build calibration method for startup.
    offset_from_east = 90 
    # imu returns clockwise values, we need it counterclockwise
    relative_angle = 360 - sensor.getVector(BNO055.VECTOR_EULER)[0]
    return (offset_from_east + relative_angle) % 360


## Publishes pose data continuously when called
def talker(data):
    ## Build nav_data msg to send
    #Target coords
    msg.tar_lat = data.target_lat
    msg.tar_long = data.target_long
    #Current coords
    msg.cur_lat =  current_coordinates['lat']
    msg.cur_long = current_coordinates['long']
    #Heading
    msg.heading = get_heading()

    # Publish pose at f rate
    pub_nav.publish(msg)

if __name__ == '__main__':
    try:
        # Make separate thread for gps
        init()
        thread_gps = Thread(target=get_coordinates)
        thread_gps.daemon=True
        thread_gps.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
