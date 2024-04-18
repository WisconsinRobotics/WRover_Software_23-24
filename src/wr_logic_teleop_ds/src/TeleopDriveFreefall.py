#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from wr_drive_msgs.msg import DriveTrainCmd
from wreadinput import DeviceAxis

rospy.init_node('TeleopDriveFreefall', anonymous=True)

drive_pub = rospy.Publisher("/control/drive_system/cmd", DriveTrainCmd, queue_size=10)

left_value = 0
right_value = 0

def left_callback(msg: Float32):
    global left_value
    left_value = -msg.data

def right_callback(msg: Float32):
    global right_value
    right_value = -msg.data

def pub_drive_freefall():

    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/hci/freefall/gamepad/axis/stick_left_y", Float32, left_callback)
    rospy.Subscriber("/hci/freefall/gamepad/axis/stick_right_y", Float32, right_callback)

    while not rospy.is_shutdown():
        drive_pub.publish(left_value, right_value)
        rate.sleep()

if __name__ == '__main__':
    try:
         pub_drive_freefall()
    except rospy.ROSInterruptException:
        pass
