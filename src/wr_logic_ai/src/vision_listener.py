#!/usr/bin/env python

import rospy
from wr_logic_ai.msg import TargetMsg

def callback(data):
    print("I heard", str(data))

def listener():
    rospy.init_node('listener')
    rospy.Subscriber('/wr_logic_ai/shortrange_ai/vision', TargetMsg, callback)
    rospy.spin()


if __name__ == "__main__":
    listener()
