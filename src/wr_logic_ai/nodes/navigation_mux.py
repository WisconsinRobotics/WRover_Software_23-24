#!/usr/bin/env python3
import rospy

long_range_nav = rospy.get_param("long_range_output")
mux_pub = rospy.Publisher('/mux', MuxPublisher, queue_size=1)

def initialize():
    rospy.Subscriber('/is_LR_complete', bool, check_LR_status)
    rospy.Subscriber('/is_LR_complete', bool, check_SR_status)

def check_LR_status(completed):
    if completed == True:
        mux_pub.publish(3) #Long range finished
    else:
        mux_pub.publish(1) #Long range is not finished

def check_SR_status(completed):
    if completed == True:
        mux_pub.publish(4) #Short range finished
    else:
        mux_pub.publish(2) #Short range is not finished
        

if __name__ == "__main__":
    rospy.init_node("navigation_multiplexer", anonymous=False)
    initialize()
    rospy.spin()