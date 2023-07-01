#!/usr/bin/env python3
# Official ROS Python code tutorial for this thing (has example code, the service server part): https://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

## @brief Provides a service to communicate with an Arduino to set LED panel colors

##@defgroup wr_led_matrix wr_led_matrix
# @{
# @defgroup led_matrix_service LED Matrix Service
# @brief Implements the LED Matrix Request service for the real hardware
#
# This uses a combination of tactics to ensure that we successfully write to the Arduino:
# * **Magic Initialization Word**: waits for a special sequence of characters to be sent by the Arudino before starting the service.  This makes sure that messages are not dropped on initialization, since the drivers for the serial port cause the Arudino to reboot on port open.
#
# * **Cyclic Redundancy Check**: Add a simple dummy value calculated from the transmitted message to the end of the sent message.  The Arudino independently calculates this checksum as well to ensure that it got the right bytes.  This decreases the chance of the message from being read off-alignment in the case of a dropped byte, since a misaligned message will be discarded.
# @{


import rospy
import serial
from wr_led_matrix.srv import led_matrix, led_matrixRequest, led_matrixResponse

## Serial line to the Arduino
s: serial.Serial = None


def handle_leds(req: led_matrixRequest):
    """Send led color to matrix via serial

    @param req The request from the ROS service
    @return An empty response object
    """
    crc = req.RED ^ req.GREEN ^ req.BLUE
    packet = bytearray([req.RED, req.GREEN, req.BLUE, crc])
    s.write(packet)
    return led_matrixResponse()


# Main executable
if __name__ == "__main__":
    rospy.init_node("led_matrix")

    ## The speed of the serial port
    baud = rospy.get_param("~baud")
    ## The port to talk to the arduino over
    port = rospy.get_param("~com_port")

    with serial.Serial(port, baud, dsrdtr=None) as s:
        while s.in_waiting < 8:
            pass
        ## The magic word from the Arduino's boot sequence
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
# @}
