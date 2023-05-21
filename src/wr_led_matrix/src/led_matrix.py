#!/usr/bin/env python3
#Official ROS Python code tutorial for this thing (has example code, the service server part): https://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
import rospy
import serial
from wr_led_matrix.srv import led_matrix,led_matrixRequest,led_matrixResponse

s: serial.Serial

def handle_leds(req: led_matrixRequest):
    """Send led color to matrix via serial

    Args:
        req (LEDMatrixRequest): _description_

    Returns:
        null
    """
    packet = bytearray([req.RED, req.GREEN, req.BLUE])
    s.write(packet)
    return led_matrixResponse()

#Main executable
if __name__=="__main__":
    rospy.init_node("led_matrix")
    rospy.Service('led_matrix', led_matrix, handle_leds)

    baud = rospy.get_param("~baud")
    port = rospy.get_param("~com_port")

    with serial.Serial(port, baud) as s:
        #ROS Spin indefinitely (the heavy lifting is in the service callback)
        rospy.loginfo("Serial port open, starting spin")
        rospy.spin()
        s.write(bytearray([0,0,0])) #Shut off LED panel
