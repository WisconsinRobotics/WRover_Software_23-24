#!/usr/bin/env python3
#Official ROS Python code tutorial for this thing (has example code, the service server part): https://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
#TODO: Import req'd things (rospy, serial, custom service, ...)

#TODO: Define a callback function that that takes in the service request and reply objects and writes to the service reply if you chose to use that optional reply data.  Please use type hints

#Main executable
if __name__=="__main__":
    #TODO: Setup the ROS service
    #ROS Spin indefinitely (the heavy lifting is in the service callback)
    rospy.spin()