#!/usr/bin/env python3
import rospy
import sensor_msgs.msg as sensor_msgs
import matplotlib.pyplot as plt
from threading import Thread

positions = None

def newJS(jsMsg: sensor_msgs.JointState):
    global positions
    if positions is None:
        positions = []
        for pos in jsMsg.position:
            positions.append([pos])
    else:
        for l, pos in zip(positions, jsMsg.position):
            l.append(pos)
            if len(l) > 500:
                del l[0]

def plot():
    global positions
    while not rospy.is_shutdown():
        if positions is not None:
            for l in positions:
                plt.plot(l)
            plt.pause(0.05)
            plt.clf()

if __name__=="__main__":
    rospy.init_node("testNode")
    sub = rospy.Subscriber("/joint_states", sensor_msgs.JointState, newJS)
    t1 = Thread(target=plot)
    t1.start()
    t1.join()
    rospy.spin()
    