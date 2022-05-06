#!/usr/bin/env python3
#Official ROS Python code tutorial for this thing (has example code, the service server part): https://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
#TODO: Import req'd things (rospy, serial, custom service, ...)
import rospy
import serial
from wr_led_matrix.srv import LEDMatrix,LEDMatrixRequest,LEDMatrixResponse

s: serial.Serial
baud = rospy.get_param("/led_matrix/baud")
port = rospy.get_param("/led_matrix/com_port")


#TODO: Define a callback function that that takes in the service request and reply objects and writes to the service reply if you chose to use that optional reply data.  Please use type hints
def handle_leds(req: LEDMatrixRequest):
    """Send led color to matrix via serial

    Args:
        req (LEDMatrixRequest): _description_

    Returns:
        null
    """
    packet = bytearray([req.RED, req.GREEN, req.BLUE])
    s.write(packet)
    return LEDMatrixResponse()

#Main executable
if __name__=="__main__":
    rospy.init_node("LEDMatrix")
    rospy.Service('LEDMatrix', LEDMatrix, handle_leds)

    with serial.Serial(port, baud) as s:
        #ROS Spin indefinitely (the heavy lifting is in the service callback)
        rospy.loginfo("Serial port open, starting spin")
        rospy.spin()
        s.write(bytearray([0,0,0])) #Shut off LED panel
