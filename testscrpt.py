#!/usr/bin/env python3
import rospy
import sensor_msgs.msg as sensor_msgs
import matplotlib.pyplot as plt
from threading import Thread

positions = None
names = None

def newJS(jsMsg: sensor_msgs.JointState):
    global positions, names
    if positions is None:
        positions = []
        names = [name for name in jsMsg.name]
        for pos in jsMsg.position:
            positions.append([pos])
    else:
        for l, pos in zip(positions, jsMsg.position):
            l.append(pos)
            if len(l) > 500:
                del l[0]

def plot():
    global positions, names
    while not rospy.is_shutdown():
        if positions is not None and names is not None:
            for l in positions:
                plt.plot(l)
            plt.legend(names)
            plt.pause(0.05)
            plt.clf()


if __name__=="__main__":
    rospy.init_node("testNode")
    sub = rospy.Subscriber("/joint_states", sensor_msgs.JointState, newJS)
    t1 = Thread(target=plot)
    t1.start()
    t1.join()
    rospy.spin()
    