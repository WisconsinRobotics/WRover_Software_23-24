#!/usr/bin/env python

import rospy
from adafruit_gps import I2C,GPS_GtopI2C as GPS
from wr_hsi_sensing.msg import CoordinateMsg
from std_msgs.msg import Float64

## Constants to vary
f = 100     # Hz | Rate at which pose messages are published

rospy.init_node('gps_testing', anonymous=False)
rate = rospy.Rate(f)
pub = rospy.Publisher('/gps_coord_data', CoordinateMsg, queue_size=1)

def cb(gps : GPS):
    gps.update()
    print(f"lat: {gps.latitude} long: {gps.longitude}")
    if gps.latitude is not None and gps.longitude is not None:
        msg = CoordinateMsg(latitude=gps.latitude, longitude=gps.longitude)
        pub.publish(msg)

if __name__ == '__main__':
    gps = GPS(i2c_bus=I2C(sda=2, scl=3))
    timer = rospy.Timer(rate.sleep_dur, lambda _: cb(gps))
    timer.start()
    rospy.spin()