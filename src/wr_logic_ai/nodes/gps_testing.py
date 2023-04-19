#!/usr/bin/env python

import rospy
from adafruit_gps import I2C,GPS_GtopI2C as GPS
from std_msgs.msg import Float64
import time

## Constants to vary
f = 100     # Hz | Rate at which pose messages are published

rospy.init_node('ai_hardware_testing', anonymous=False)
rate = rospy.Rate(f)
pub_lat = rospy.Publisher('/latitude', Float64, queue_size=1)
pub_long = rospy.Publisher('/longitude', Float64, queue_size=1)

def cb(gps : GPS):
    gps.update()
    if gps.latitude is not None and gps.longitude is not None:
        pub_lat.publish(gps.latitude)
        pub_long.publish(gps.longitude)

if __name__ == '__main__':
    gps = GPS(i2c_bus=I2C(sda=2, scl=3))
    timer = rospy.Timer(rate.sleep_dur, lambda _: cb())
    rospy.spin()