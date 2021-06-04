#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16

from wr_drive_msgs.msg import CamMastCmd

def main():
    rospy.init_node('cam_mast_control')

    pub = rospy.Publisher('/hsi/roboclaw/aux3/cmd/right', Int16)
    
    def msg_cb(msg: CamMastCmd):
        pub.publish(Int16(round(msg.turn_speed * 32767)))
    
    sub = rospy.Subscriber('/control/camera/cam_mast_cmd', CamMastCmd, msg_cb)
    
    rospy.spin()

if __name__ == '__main__':
    main()
