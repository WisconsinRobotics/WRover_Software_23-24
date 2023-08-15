#!/usr/bin/env python3

## @defgroup wr_control_drive_ds_cam Camera Control
# @brief Translates logical speeds to hardware speeds for the camera mast
# @ingroup wr_control_drive_ds
# @{

## @file
# @author Evan Geng

import rospy
from std_msgs.msg import Int16

from wr_drive_msgs.msg import CamMastCmd

def main():
    """
    A function acting as the entrypoint of the camera mast node
    """
    
    rospy.init_node('cam_mast_control')

    pub = rospy.Publisher('/hsi/roboclaw/aux3/cmd/right', Int16, queue_size=4)
    
    def msg_cb(msg: CamMastCmd):
        pub.publish(Int16(round(msg.turn_speed * 32767)))
    
    sub = rospy.Subscriber('/control/camera/cam_mast_cmd', CamMastCmd, msg_cb)
    
    rospy.spin()

if __name__ == '__main__':
    main()

## @}