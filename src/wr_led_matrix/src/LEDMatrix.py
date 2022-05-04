#!/usr/bin/env python3
#Official ROS Python code tutorial for this thing (has example code, the service server part): https://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
#TODO: Import req'd things (rospy, serial, custom service, ...)
import rospy
import serial
import time
from wr_led_matrix.srv import LEDMatrix,LEDMatrixRequest,LEDMatrixResponse

s: serial.Serial

#TODO: Define a callback function that that takes in the service request and reply objects and writes to the service reply if you chose to use that optional reply data.  Please use type hints
def handle_leds(req: LEDMatrixRequest):
    p = bytearray([req.RED, req.GREEN, req.BLUE])
    s.write(p)
    return LEDMatrixResponse()
    
#Main executable
if __name__=="__main__":
    #TODO: Setup the ROS service
    rospy.init_node("LEDMatrix")
    rospy.Service('LEDMatrix', LEDMatrix, handle_leds)

    with serial.Serial('/dev/ttyUSB1', 9600) as s:
        #ROS Spin indefinitely (the heavy lifting is in the service callback)
        rospy.spin()
        s.write(bytearray([0,0,0])) #Shut off LED panel