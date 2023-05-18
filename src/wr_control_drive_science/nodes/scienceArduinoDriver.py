#!/usr/bin/env python3

from serial import Serial
from threading import Lock
import struct
import rospy
import std_msgs.msg as std_msgs

# bytepackage format
SERVO_SERIAL_PACKET_FORMAT = "B"
MOISTURE_SENSOR_PACKET_FORMAT = "I"

# Storage for the most recent servo command
servo_position = 0
servo_position_lock = Lock()

# Storage for the most recent sensor readings (most recent only)
moisture_sensor_reading = 0
moisture_sensor_lock = Lock()


def get_moisture() -> int:
    # Get the moisture sensor reading from hardware
    pass


def set_servo_position(position: int) -> None:
    # Set the cache servo position in hardware
    pass


def arduinoSerialProcessing(ser: Serial) -> None:
    # expect call from ROS timer
    global servo_position
    global servo_position_lock
    global moisture_sensor_reading
    global moisture_sensor_lock
    with ser as serial_instance:
        curr_moisture = get_moisture()
        with moisture_sensor_lock:
            moisture_sensor_reading = curr_moisture

        next_servo_position = 0
        with servo_position_lock:
            next_servo_position = servo_position
        set_servo_position(next_servo_position)


def ros_subscriber_servo(msg: std_msgs.UInt8) -> None:
    with servo_position_lock:
        servo_position = msg.data


def ros_publish_moisture(pub: rospy.Publisher) -> None:
    moisture = 0
    with moisture_sensor_lock:
        moisture = moisture_sensor_reading
    pub.publish(std_msgs.UInt32(data=moisture))


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
    moisture_publisher = rospy.Publisher(
        "/moisture", std_msgs.UInt32, queue_size=1)
    rospy.Timer(rospy.Duration.from_sec(0.1),
                lambda _: ros_publish_moisture(moisture_publisher))

    # Arduino comms loop
    rospy.Timer(rospy.Duration.form_sec(0.1),
                lambda _: arduinoSerialProcessing(ser))


if __name__ == "__main__":
    initialize()
    rospy.spin()
