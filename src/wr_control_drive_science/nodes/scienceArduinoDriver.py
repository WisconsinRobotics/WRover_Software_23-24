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
import std_msgs.msg as std_msgs
import time

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


def ros_publish_moisture(
    volWaterPub: rospy.Publisher,
    temperaturePub: rospy.Publisher,
    conductivityPub: rospy.Publisher,
) -> None:
    """Publishes cached moisture data over ROS topics

    @param volWaterPub The publisher for volumetric water data
    @param temperaturePub The publisher for temperature
    @param conductivityPub The publisher for soil conductivity
    """

    global vol_water_reading
    global temperature_reading
    global conductivity_reading
    global moisture_sensor_lock
    vol_water = 0
    temperature = 0
    conductivity = 0
    with moisture_sensor_lock:
        vol_water = vol_water_reading
        temperature = temperature_reading
        conductivity = conductivity_reading
    volWaterPub.publish(std_msgs.Float32(data=vol_water))
    temperaturePub.publish(std_msgs.Float32(data=temperature))
    conductivityPub.publish(std_msgs.Float32(data=conductivity))


def initialize() -> None:
    """Set up the ROS interface and callbacks to run the node"""

    rospy.init_node("scienceArduinoDriver")
    # No reasonable default, must be supplied
    serial_name = rospy.get_param("~serial_name")
    serial_baud = rospy.get_param("~serial_baud", 115200)

    ser = Serial(port=serial_name, baudrate=serial_baud)
    # Moisture publishing
    vol_water_publisher = rospy.Publisher("/vol_water", std_msgs.Float32, queue_size=1)
    temperature_publisher = rospy.Publisher(
        "/temperature", std_msgs.Float32, queue_size=1
    )
    conductivity_publisher = rospy.Publisher(
        "/conductivity", std_msgs.Float32, queue_size=1
    )
    rospy.Timer(
        rospy.Duration.from_sec(0.1),
        lambda _: ros_publish_moisture(
            vol_water_publisher, temperature_publisher, conductivity_publisher
        ),
    )

    # Arduino comms loop
    rospy.Timer(rospy.Duration.from_sec(0.1), lambda _: arduinoSerialProcessing(ser))


if __name__ == "__main__":
    initialize()
    rospy.spin()

## @}
# @}
