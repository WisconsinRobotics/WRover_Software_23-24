#!/usr/bin/env python3
# Official ROS Python code tutorial for this thing (has example code, the service server part): https://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

## @brief Provides a service to communicate with an Arduino to set LED panel colors

##@defgroup wr_led_matrix wr_led_matrix
# @{


import rospy
import serial
from wr_led_matrix.srv import led_matrix, led_matrixRequest, led_matrixResponse

## Serial line to the Arduino
s: serial.Serial = None


def handle_leds(req: led_matrixRequest):
    """Send led color to matrix via serial

    Args:
        req (LEDMatrixRequest): _description_

    Returns:
        null
    """
    crc = req.RED ^ req.GREEN ^ req.BLUE
    packet = bytearray([req.RED, req.GREEN, req.BLUE, crc])
    s.write(packet)
    return led_matrixResponse()


# Main executable
if __name__ == "__main__":
    rospy.init_node("led_matrix")

    baud = rospy.get_param("~baud")
    port = rospy.get_param("~com_port")

    with serial.Serial(port, baud, dsrdtr=None) as s:
        while s.in_waiting < 8:
            pass
        magic_word = s.read(8)
        if magic_word != bytes([0x31, 0x41, 0x59, 0x26, 0xDE, 0xAD, 0xBE, 0xEF]):
            rospy.logfatal(
                f"Received incorrect magic word ({magic_word}) on startup, exiting..."
            )
            exit(1)
        rospy.Service("led_matrix", led_matrix, handle_leds)
        # ROS Spin indefinitely (the heavy lifting is in the service callback)
        rospy.loginfo("Serial port open, starting spin")
        rospy.spin()
        s.write(bytearray([0, 0, 0, 0]))  # Shut off LED panel

## @}
