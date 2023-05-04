#!/usr/bin/env python3
import rospy
import std_msgs.msg
import math

timestep = 0
PERIOD = 10

def do_timestep_publications(event:rospy.timer.TimerEvent, leftPub : rospy.Publisher, rightPub : rospy.Publisher) -> None:
    global timestep
    ## Walk the motor speeds
    lpower = math.sin(timestep * 2 * math.pi / 10)
    rpower = math.sin(timestep * 2 * math.pi / 10 + math.pi/4)
    leftPub.publish(std_msgs.msg.Float64(data=lpower))
    rightPub.publish(std_msgs.msg.Float64(data=-rpower))

    timestep += 1/50.0
    timestep %= PERIOD * 1000

if __name__ == "__main__":
    rospy.init_node("test_drive")
    leftPub = rospy.Publisher("/left/power", std_msgs.msg.Float64)
    rightPub = rospy.Publisher("/right/power", std_msgs.msg.Float64)
    timer = rospy.Timer(rospy.Duration(1/50.0), lambda event : do_timestep_publications(event, leftPub, rightPub))
    rospy.spin()
