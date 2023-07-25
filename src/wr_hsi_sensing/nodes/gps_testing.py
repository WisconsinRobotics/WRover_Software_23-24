#!/usr/bin/env python

## @file
# @brief Node executing GPS and heading calculations
# @ingroup wr_hsi_sensing
# @defgroup wr_hsi_sensing_gps GPS
# @brief Responsible for calculating GPS and heading data
# @details Getting coordinate data from the GPS is a straightforward query to the sensor.  Calculating the heading from GPS data requires
# reading multiple GPS positions and finding the heading of the line connecting the two GPS coordinates.  This method works well for systems that
# have consistent motion patterns and can really only move in 1 direction (like cars/trucks).  This is not a great candidate for calculating heading on
# the rover; it is a better idea to get the @ref wr_hsi_sensing_imu working.
# @ingroup wr_hsi_sensing
# @{

import math
import rospy
from adafruit_gps import I2C, GPS_GtopI2C as GPS
from wr_hsi_sensing.msg import CoordinateMsg
from std_msgs.msg import Float64

# Constants to vary
## Rate at which pose messages are published in hz
f = 100

## @cond
rospy.init_node("gps_testing", anonymous=False)
## @endcond
## Rate controlling how frequently we publish data
rate = rospy.Rate(f)
## Publisher for GPS coordinate data
pub = rospy.Publisher("/gps_coord_data", CoordinateMsg, queue_size=1)
## Publisher for heading data
# TODO(@bennowotny): This should be migrated back to the IMU code
pub_heading = rospy.Publisher("/heading_data", Float64, queue_size=1)

## Last latitude recorded, used to calculate heading
last_lat = None
## Last longitude recorded, used to calculate heading
last_long = None
## Last heading recorded, used to maintain publishing frequency once heading is locked
cached_heading = None

## How much the GPS data has to deviate to reliably calculate heading
GPS_TOLERANCE = 0.000001

"""
Optional: average heading to get more stable value
NUM_CACHED_HEADINGS = 3
heading_ind = 0
past_headings = np.zeros(NUM_CACHED_HEADINGS)
"""


def cb(gps: GPS) -> None:
    """ROS Timer callback to process data (coordinates and heading) from the GPS sensor

    @param gps The GPS sensor to talk to
    """
    try:
        gps.update()
    except:
        return
    print(f"lat: {gps.latitude} long: {gps.longitude}")
    if gps.latitude is not None and gps.longitude is not None:
        global last_lat
        global last_long
        global cached_heading
        """
        Optional: average heading to get more stable value
        global heading_ind
        global past_headings
        """

        msg = CoordinateMsg(latitude=gps.latitude, longitude=gps.longitude)
        pub.publish(msg)

        # TODO (@bennowotny): We probably want to keep this, but it should be in its own file, or at least an option we can shut off
        if (
            last_lat is not None
            and last_long is not None
            and (
                abs(gps.longitude - last_long) > GPS_TOLERANCE
                or abs(gps.latitude - last_lat) > GPS_TOLERANCE
            )
        ):
            # Find heading from the change in latitude and longitude
            X = math.cos(gps.latitude) * math.sin(gps.longitude - last_long)
            Y = math.cos(last_lat) * math.sin(gps.latitude) - math.sin(
                last_lat
            ) * math.cos(gps.latitude) * math.cos(gps.longitude - last_long)
            bearing = math.atan2(X, Y)
            bearing = bearing if bearing > 0 else 2 * math.pi + bearing
            bearing = math.degrees(bearing)
            cached_heading = bearing
            pub_heading.publish(Float64(data=bearing))

        """
        Optional: average heading to get more stable value
        past_headings[heading_ind] = bearing
        i += 1
        i %= NUM_CACHED_HEADINGS
        avg_heading = past_headings.sum() / NUM_CACHED_HEADINGS
        pub_heading.publish(Float64(data=avg_heading))
        """

        # Update last GPS latitude and longitude
        last_lat = gps.latitude
        last_long = gps.longitude
    if cached_heading is not None:
        pub_heading.publish(Float64(data=cached_heading))


if __name__ == "__main__":
    ## GPS used to get data, defined on the RPi I2C bus
    gps = GPS(i2c_bus=I2C(sda=2, scl=3))
    gps.begin(0x10)
    ## ROS timer to calculate and publish GPS data
    timer = rospy.Timer(rate.sleep_dur, lambda _: cb(gps))
    rospy.spin()

## @}
