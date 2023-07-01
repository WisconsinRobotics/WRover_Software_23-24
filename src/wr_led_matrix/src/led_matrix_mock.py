#!/usr/bin/env python3
# Official ROS Python code tutorial for this thing (has example code, the service server part): https://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
import rospy

# import serial
from wr_led_matrix.srv import led_matrix, led_matrixRequest, led_matrixResponse

## @defgroup wr_led_matrix
# @{
# @defgroup led_matrix_service_mock Mock LED Matrix Service
# @brief A mock implementation of the LED Matrix ROS service
#
# This application creates a service with an identical interface to the real LED Matrix service, but does not attempt to communicate with any hardware; it just swallows the message.  This allows other applications to test service calls without needing actual hardware
# @{

# s: serial.Serial


def handle_leds(_: led_matrixRequest):
    """Send led color to matrix via serial

    @param req The request from the ROS service.  This is unused in mock mode
    @return An empty response object
    """
    # packet = bytearray([req.RED, req.GREEN, req.BLUE])
    # s.write(packet)
    return led_matrixResponse()


# Main executable
if __name__ == "__main__":
    rospy.init_node("led_matrix")
    rospy.Service("led_matrix", led_matrix, handle_leds)

    # baud = rospy.get_param("~baud")
    # port = rospy.get_param("~com_port")

    # with serial.Serial(port, baud, dsrdtr=None) as s:
    # ROS Spin indefinitely (the heavy lifting is in the service callback)
    # rospy.loginfo("Serial port open, starting spin")
    rospy.spin()
    # s.write(bytearray([0,0,0])) #Shut off LED panel

## @} @}
