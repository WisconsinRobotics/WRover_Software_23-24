#!/usr/bin/env python3

##@addtogroup wr_control_drive_science
# @{
# @defgroup wr_control_drive_science_python_arduino Arduino-to-ROS Interface
# @brief Streams data between the science Arduino (@ref wr_control_drive_science/arduino/arduino.ino "arduino.ino (science)") and ROS.
# @details This may be replaceable by ROSserial at some point in the future.
# @{
# @file
# @brief ROS interface for the Arduino code in @ref wr_control_drive_science/arduino/arduino.ino "arduino.ino (science)".

from typing import Tuple, Optional
from serial import Serial
from threading import Lock
import struct
import rospy

import time

from wr_control_drive_science.srv import ScienceService
from wr_control_drive_science.srv import ScienceServiceRequest
from wr_control_drive_science.srv import ScienceServiceResponse

## Moisture sensor data format
MOISTURE_SENSOR_PACKET_FORMAT = "<ffh"

## Unit conversion for conductivity
COND_UNIT = 1

# Storage for the most recent sensor readings (most recent only)
## Reading for volumetric water content
vol_water_reading = 0
## Reading for temperature
temperature_reading = 0
## Reading for conductivity
conductivity_reading = 0
## Lock to avoid synchronization issues
moisture_sensor_lock = Lock()


def get_moisture(ser: Serial) -> Tuple[Optional[int]]:
    """Get moisture sensor data from a serial connection

    @param ser The serial connection to get data from

    @return The data for volumetric water content, temperature, and conductivity if available.  Otherwise, @code [None]*3 @endcode
    """

    # Get the moisture sensor reading from hardware
    print(f"in waiting: {ser.in_waiting}")
    if ser.in_waiting >= struct.calcsize(MOISTURE_SENSOR_PACKET_FORMAT) + 4:
        m = ser.read(4)
        if m != bytearray([0xAA, 0xAA, 0xAA, 0xAA]):
            return None, None, None
        new_vol_water, new_temperature, new_conductivity = struct.unpack(
            MOISTURE_SENSOR_PACKET_FORMAT,
            ser.read(struct.calcsize(MOISTURE_SENSOR_PACKET_FORMAT)),
        )
        return new_vol_water, new_temperature, new_conductivity * COND_UNIT
    return None, None, None


def arduinoSerialProcessing(ser: Serial) -> None:
    """A function that updates the global moisture sensor variables when data is available

    @param ser The serial connection to use
    """

    # expect call from ROS timer
    global vol_water_reading
    global temperature_reading
    global conductivity_reading
    global moisture_sensor_lock
    curr_vol_water, curr_temperature, curr_conductivity = get_moisture(ser)
    # curr_vol_water, curr_temperature, curr_conductivity = 0,0,0
    with moisture_sensor_lock:
        vol_water_reading = (
            curr_vol_water if curr_vol_water is not None else vol_water_reading
        )
        temperature_reading = (
            curr_temperature if curr_temperature is not None else temperature_reading
        )
        conductivity_reading = (
            curr_conductivity if curr_conductivity is not None else conductivity_reading
        )

def clientHandler(request: ScienceServiceRequest):
    response = ScienceServiceResponse(
        vol_water_reading=str(vol_water_reading),
        temperature_reading=str(temperature_reading),
        conductivity_reading=str(conductivity_reading)
    )

    return response



def initialize() -> None:
    """Set up the ROS interface and callbacks to run the node"""

    rospy.init_node("scienceArduinoDriver")
    # No reasonable default, must be supplied
    serial_name = rospy.get_param("~serial_name")
    serial_baud = rospy.get_param("~serial_baud", 115200)

    ser = Serial(port=serial_name, baudrate=serial_baud)
    # ser = None

    # Arduino comms loop
    rospy.Timer(rospy.Duration.from_sec(0.1), lambda _: arduinoSerialProcessing(ser))

    s = rospy.Service("science_service", ScienceService, clientHandler)
    rospy.loginfo("OOO")



if __name__ == "__main__":
    initialize()
    rospy.spin()

## @}
# @}
