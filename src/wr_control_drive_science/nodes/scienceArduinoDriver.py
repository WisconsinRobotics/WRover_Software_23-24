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
import re
import rospy
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

import time

from wr_control_drive_science.srv import ScienceService
from wr_control_drive_science.srv import ScienceServiceRequest
from wr_control_drive_science.srv import ScienceServiceResponse

from wr_control_drive_science.msg import LightMsg

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

light_DNA_reading = 0
light_protein_reading = 0


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
    global light_DNA_reading
    global vol_water_reading
    global conductivity_reading
    global temperature_reading
    global moisture_sensor_lock
    # global light_protein_reading

    with moisture_sensor_lock:
        if ser.in_waiting > 25:
            ser.reset_input_buffer()
            ser.readline()

        data_str = ser.readline().decode()
        data_arr = re.match(r'(\+|-)', data_str)
        if len(data_arr) == 7:
            light_DNA_reading = float(data_arr[0])
            vol_water_reading = (1 if data_arr[1] == '+' else -1) * float(data_arr[2])
            conductivity_reading = (1 if data_arr[3] == '+' else -1) * float(data_arr[4])
            temperature_reading = (1 if data_arr[5] == '+' else -1) * float(data_arr[6])


def lightProteinProcessing(ser: Serial):
    global light_protein_reading
    if ser.in_waiting > 10:
        ser.reset_input_buffer()
        ser.readline()

    light_protein_reading = float(ser.readline())


def clientHandler(request: ScienceServiceRequest):
    response = ScienceServiceResponse(
        vol_water_reading=str(vol_water_reading),
        temperature_reading=str(temperature_reading),
        conductivity_reading=str(conductivity_reading)
    )
    return response

def publishLightData(pubLightSensor):
    msg = LightMsg(
        LightDNA = light_DNA_reading,
        LightProtein = light_protein_reading
    )
    pubLightSensor.publish(msg)

def handleServo(ser: Serial):
    global moisture_sensor_lock

    with moisture_sensor_lock:
        rospy.loginfo("<<<<<<<<<Moving servo>>>>>>>>")
        ser.write(b'W')

def initialize() -> None:
    """Set up the ROS interface and callbacks to run the node"""

    rospy.init_node("scienceArduinoDriver")
    # No reasonable default, must be supplied
    serial_name = rospy.get_param("~serial_name")
    serial_baud = rospy.get_param("~serial_baud", 115200)

    ser = Serial(port=serial_name, baudrate=serial_baud)

    light_sensor_serial_name = rospy.get_param("~light_sensor_serial_name")
    light_sensor_serial_baud = rospy.get_param("~lignt_sensor_serial_baud", 115200)

    light_sensor_ser = Serial(port=light_sensor_serial_name, baudrate=light_sensor_serial_baud)
    # ser = None

    # Arduino comms loop
    rospy.Timer(rospy.Duration.from_sec(0.1), lambda _: arduinoSerialProcessing(ser))
    rospy.Timer(rospy.Duration.from_sec(0.1), lambda _: lightProteinProcessing(light_sensor_ser))

    pubLightSensor = rospy.Publisher('light_data', LightMsg, queue_size=10)

    science_data_service = rospy.Service("science_service", ScienceService, clientHandler)
    move_servo_service = rospy.Service("move_servo", Empty, lambda _: handleServo(ser))

    send_light_data = rospy.Timer(
        rospy.Duration.from_sec(1), lambda _: publishLightData(pubLightSensor)
    )


if __name__ == "__main__":
    initialize()
    rospy.spin()

## @}
# @}
