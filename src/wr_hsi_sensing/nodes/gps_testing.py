#!/usr/bin/env python

import math
import numpy as np
import rospy
from adafruit_gps import I2C,GPS_GtopI2C as GPS
from wr_hsi_sensing.msg import CoordinateMsg
from std_msgs.msg import Float64

## Constants to vary
f = 100     # Hz | Rate at which pose messages are published

rospy.init_node('gps_testing', anonymous=False)
rate = rospy.Rate(f)
pub = rospy.Publisher('/gps_coord_data', CoordinateMsg, queue_size=1)
pub_heading = rospy.Publisher('/heading_data', Float64, queue_size=1)

last_lat = None
last_long = None

'''
Optional: average heading to get more stable value
NUM_CACHED_HEADINGS = 3
heading_ind = 0
past_headings = np.zeros(NUM_CACHED_HEADINGS)
'''

def cb(gps : GPS):
    gps.update()
    print(f"lat: {gps.latitude} long: {gps.longitude}")
    if gps.latitude is not None and gps.longitude is not None:
        global last_lat
        global last_long
        '''
        Optional: average heading to get more stable value
        global heading_ind
        global past_headings
        '''

        msg = CoordinateMsg(latitude=gps.latitude, longitude=gps.longitude)
        pub.publish(msg)

        # Find heading from the change in latitude and longitude
        X = math.cos(gps.latitude) * math.sin(gps.longitude - last_long)
        Y = math.cos(last_lat) * math.sin(gps.latitude) \
            - math.sin(last_lat) * math.cos(gps.latitude) * math.cos(gps.longitude - last_long)
        bearing = math.atan2(X, Y)
        pub_heading.publish(Float64(data=bearing))

        '''
        Optional: average heading to get more stable value
        past_headings[heading_ind] = bearing
        i += 1
        i %= NUM_CACHED_HEADINGS
        avg_heading = past_headings.sum() / NUM_CACHED_HEADINGS
        pub_heading.publish(Float64(data=avg_heading))
        '''

        # Update last GPS latitude and longitude
        last_lat = gps.latitude
        last_long = gps.longitude

if __name__ == '__main__':
    gps = GPS(i2c_bus=I2C(sda=2, scl=3))
    gps.begin(0x10)
    timer = rospy.Timer(rate.sleep_dur, lambda _: cb(gps))
    rospy.spin()