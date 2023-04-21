#!/usr/bin/env python

import rospy
from wr_hsi_sensing.BNO055 import BNO055
from std_msgs.msg import Int32, Float64
import math

## Constants to vary
f = 100     # Hz | Rate at which pose messages are published

rospy.init_node('ai_hardware_testing', anonymous=False)
rate = rospy.Rate(f)
pub_x = rospy.Publisher('/mag_x', Int32, queue_size=1)
pub_y = rospy.Publisher('/mag_y', Int32, queue_size=1)
pub_z = rospy.Publisher('/mag_z', Int32, queue_size=1)

pub_norm_x = rospy.Publisher('/norm_mag_x', Int32, queue_size=1)
pub_norm_y = rospy.Publisher('/norm_mag_y', Int32, queue_size=1)
pub_heading = rospy.Publisher('/heading', Float64, queue_size=1)

MAG_NOISE_THRESH = 10000

min_x = float('inf')
max_x = float('-inf')
min_y = float('inf')
max_y = float('-inf')
prev_x = 0
prev_y = 0

# Create IMU Sensor object
sensor = BNO055()
if not sensor.begin(mode=BNO055.OPERATION_MODE_MAGONLY):
    raise RuntimeError("IMU Failed to initialize")
sensor.setExternalCrystalUse(True)
# Dump some initial IMU readings
# imuInitCounter = 0
# while((sensor.getVector(BNO055.VECTOR_EULER)[2] == 0.0)and
#         (imuInitCounter > 100)):
#     print(sensor.getVector(BNO055.VECTOR_EULER))
#     rate.sleep()
#     imuInitCounter+=1

## Retrieve heading from IMU (Returns a float angle)
def get_heading():
    # TODO: get actual obset from an absolute value ---- Build calibration method for startup.
    offset_from_east = 90 
    # imu returns clockwise values, we need it counterclockwise
    relative_angle = 360 - sensor.getVector(BNO055.VECTOR_EULER)[0]
    return (offset_from_east + relative_angle) % 360

def cb():
    global max_x
    global min_x
    global max_y
    global min_y
    global prev_x
    global prev_y

    mag_z = int.from_bytes(sensor.readBytes(BNO055.BNO055_MAG_DATA_Z_LSB_ADDR, 2), 'little')
    mag_x = int.from_bytes(sensor.readBytes(BNO055.BNO055_MAG_DATA_X_LSB_ADDR, 2), 'little')
    mag_y = int.from_bytes(sensor.readBytes(BNO055.BNO055_MAG_DATA_Y_LSB_ADDR, 2), 'little')

    mag_x = mag_x if mag_x < 32768 else mag_x-65536
    mag_y = mag_y if mag_y < 32768 else mag_y-65536
    mag_z = mag_z if mag_z < 32768 else mag_z-65536

    if abs(mag_x - prev_x) < MAG_NOISE_THRESH and abs(mag_y - prev_y) < MAG_NOISE_THRESH:
        max_x = max(max_x, mag_x)
        min_x = min(min_x, mag_x)
        max_y = max(max_y, mag_y)
        min_y = min(min_y, mag_y)
        prev_x = mag_x
        prev_y = mag_y

        norm_x = mag_x - (max_x - min_x) / 2
        norm_y = mag_y - (max_y - min_y) / 2
        pub_norm_x.publish(int(norm_x))
        pub_norm_y.publish(int(norm_y))

        heading = math.atan2(norm_y, norm_x)
        pub_heading.publish(Float64(heading))

    mag_z = mag_z if abs(mag_z) < 10000 else 0

    pub_x.publish(mag_x)
    pub_y.publish(mag_y)
    pub_z.publish(mag_z)

if __name__ == '__main__':
    prev_x = int.from_bytes(sensor.readBytes(BNO055.BNO055_MAG_DATA_X_LSB_ADDR, 2), 'little')
    prev_y = int.from_bytes(sensor.readBytes(BNO055.BNO055_MAG_DATA_Y_LSB_ADDR, 2), 'little')
    prev_x = prev_x if prev_x < 32768 else prev_x-65536
    prev_y = prev_y if prev_y < 32768 else prev_y-65536
    timer = rospy.Timer(rate.sleep_dur, lambda _: cb())
    rospy.spin()