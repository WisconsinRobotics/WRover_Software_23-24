#!/usr/bin/env python3

from typing import Optional
from serial import Serial
from threading import Lock
import struct
import rospy
import std_msgs.msg as std_msgs

# bytepackage format
SERVO_SERIAL_PACKET_FORMAT = "B"
MOISTURE_SENSOR_PACKET_FORMAT = "fff"

# Storage for the most recent servo command
servo_position = 0
servo_position_lock = Lock()

# Storage for the most recent sensor readings (most recent only)
vol_water_reading = 0
temperature_reading = 0
conductivity_reading = 0
moisture_sensor_lock = Lock()

def get_moisture(ser: Serial) -> Optional[int]:
    # Get the moisture sensor reading from hardware
    if ser.in_waiting >= struct.calcsize(MOISTURE_SENSOR_PACKET_FORMAT):
        new_vol_water, new_temperature, new_conductivity = struct.unpack(MOISTURE_SENSOR_PACKET_FORMAT, ser.read(
            struct.calcsize(MOISTURE_SENSOR_PACKET_FORMAT)))
        return new_vol_water, new_temperature, new_conductivity
    return None, None, None


def set_servo_position(position: int, ser: Serial) -> None:
    # Set the cache servo position in hardware
    ser.write(struct.pack(SERVO_SERIAL_PACKET_FORMAT, position))


def arduinoSerialProcessing(ser: Serial) -> None:
    # expect call from ROS timer
    global servo_position
    global servo_position_lock
    global vol_water_reading
    global temperature_reading
    global conductivity_reading
    global moisture_sensor_lock
    with ser as serial_instance:
        curr_vol_water, curr_temperature, curr_conductivity = get_moisture(serial_instance)
        with moisture_sensor_lock:
            vol_water_reading = curr_vol_water if curr_vol_water is not None else vol_water_reading
            temperature_reading = curr_temperature if curr_temperature is not None else temperature_reading
            conductivity_reading = curr_conductivity if curr_conductivity is not None else conductivity_reading

        next_servo_position = 0
        with servo_position_lock:
            next_servo_position = servo_position
        set_servo_position(next_servo_position, serial_instance)


def ros_subscriber_servo(msg: std_msgs.UInt8) -> None:
    global servo_position
    global servo_position_lock
    # Ensure that the publisher's data is in the range [0,180]
    with servo_position_lock:
        servo_position = msg.data


def ros_publish_moisture(volWaterPub: rospy.Publisher, temperaturePub: rospy.Publisher, conductivityPub: rospy.Publisher) -> None:
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
    rospy.init_node("scienceArduinoDriver")
    # No reasonable default, must be supplied
    serial_name = rospy.get_param("~serial_name")
    serial_baud = rospy.get_param("~serial_baud", 115200)

    ser = Serial(port=serial_name, baudrate=serial_baud)

    # Servo subscriber
    rospy.Subscriber("/servo_position", std_msgs.UInt8,
                     ros_subscriber_servo, queue_size=1)

    # Moisture publishing
    vol_water_publisher = rospy.Publisher(
        "/vol_water", std_msgs.Float32, queue_size=1)
    temperature_publisher = rospy.Publisher(
        "/temperature", std_msgs.Float32, queue_size=1)
    conductivity_publisher = rospy.Publisher(
        "/conductivity", std_msgs.Float32, queue_size=1)
    rospy.Timer(rospy.Duration.from_sec(0.1),
                lambda _: ros_publish_moisture(vol_water_publisher, temperature_publisher, conductivity_publisher))

    # Arduino comms loop
    rospy.Timer(rospy.Duration.from_sec(0.1),
                lambda _: arduinoSerialProcessing(ser))


if __name__ == "__main__":
    initialize()
    rospy.spin()
