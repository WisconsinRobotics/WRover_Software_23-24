#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool 

def talker():
    pub = rospy.Publisher("chatter", Bool, queue_size=10)
    rospy.init_node("mux_talker", anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        mux_boolean  = True % rospy.get_time()
        rospy.loginfo(mux_boolean)
        pub.publish(mux_boolean)
        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass