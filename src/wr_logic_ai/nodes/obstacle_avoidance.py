#!/usr/bin/env python3

"""@file
@defgroup wr_logic_ai_longrange_ai
@{
@defgroup wr_logic_ai_longrange_ai_obstacle_avoidance Obstacle Avoidance
@brief Driver code for long range navigation logics
@details Navigates the rover to the target coordinate using the rover's current coordinates and 
a target coordinate. This code also takes obstacle avoidance into account, by calculating an 
"open window" that is closest towards the target heading and navigating towards that window. 
@{
"""

import math
import rospy
import time
from finder import get_navigation_angle
from angle_calculations import AngleCalculations
import angle_to_drive_methods as angle_calc

from sensor_msgs.msg import LaserScan

from wr_hsi_sensing.msg import CoordinateMsg
from wr_drive_msgs.msg import DriveTrainCmd
from std_msgs.msg import Float64

# Navigation parameters
# distance before obstacle avoidance logics is triggered (in meters)
LIDAR_THRESH_DISTANCE = 2.5
# distance before rover believes it has reached the target (in meters)
NAV_THRESH_DISTANCE = 0.5

# initialize target angle to move forward
current_lat = 10
current_long = 0

##Value robot is heading currently. East is 0. (Counterclockwise). Extected as a value from 0 to 360.
cur_heading = 0
##Gives target angle relative to East (counter-clockwise)
target_angle = 270
target_lat = 0
target_long = 0

##Used to smooth over lidar data
smoothing_constant = 3
##Speed multiplier for long range navigation
# 0.2 is bugged
speed_factor = 0
##Most recent data from lidar module
lidar_data = LaserScan()
##If the rover is running in long range navigation
is_active = False

def initialize() -> None:
    """Initialize publisher and subscribers for nodes

    Publishers:
        drive_pub: Sends motor speeds to robot
        raw_heading_pug: Publishes data of our current heading relative to the right of the robot. (90 Degrees is where the robot is pointing)
        heading_pub: Data from heading is transformed to be worked with rviz tester.

    Subscriber:
        gps_coord_data: gets lat and long coordinates from GPS
        heading_data: Gets data of our heading with East beign 0.
        scan: Get values from lidar scan

    """
    global drive_pub
    global smoothing_constant
    global speed_factor

    # Publisher
    drive_pub = rospy.Publisher("/control/drive_system/cmd", DriveTrainCmd, queue_size=1)

    # Subscribe to gps coordinate data
    rospy.Subscriber("/gps_coord_data", CoordinateMsg, update_gps_coord)

    # Subscribe to heading data
    rospy.Subscriber("/heading_data", Float64, update_heading)

    # Subscribe to lidar data
    rospy.Subscriber('/scan', LaserScan, update_lidar_data)
    
def set_is_active(active: bool) -> None:
    """
    Setting the active state for obstacle avoidance

    @param active (bool) True implies that the rover is running in long range mode, and false otherwise
    """
    global is_active
    is_active = active

def update_gps_coord(msg: CoordinateMsg) -> None:
    """
    Updates stored rover GPS coordiantes to the newest reading

    @param msg (CoordinateMsg) Data received from GPS module
    """
    global current_lat
    global current_long
    current_lat = msg.latitude
    current_long = msg.longitude

def update_heading(msg: Float64) -> None:
    """
    Shifted from compass coordinates to math coordinates.

    @param msg (Float64): Heading value received as value from 0 to 360. North is 0 clockwise.
    @param cur_heading East is 0. (Counterclockwise). Extected as a value from 0 to 360.
    """
    global cur_heading
    cur_heading = (90 - msg.data) % 360  # Shifting to East

def angle_diff(heading1: float, heading2: float) -> float:
    """
    Returns relative angle difference from heading of robot to heading of target

    @param heading1 (float): Value of target relative to East (Counter-clockwise)
    @param heading2 (float): Value of heading relative to East (Counter-clockwise)
    @return float: Value from -180 to 180. Starting from bottom of robot (Counter-Clockwise)
    """
    diff = (heading1 - heading2 + 360) % 360
    return (diff + 180) % 360 - 180


# Calculate current heading and the planar target angle
# TODO: this should now be part of a action server callback function
def update_target(lat, long) -> None:
    """
    Updates target_angle with values from IMU which use gps coords

    @param target_angle: Gives target angle relative to East (counter-clockwise)
    @param imu An object that creates a planar target angle relative to east, accounting for curvature of Earth
    @return bool: If we have arrived to the destination, based on our current and target coords
    """
    global target_angle
    global target_lat
    global target_long

    # Construct the planar target angle relative to east, accounting for curvature
    imu = AngleCalculations(current_lat, current_long,
                            lat, long)

    target_angle = imu.get_angle() % 360
    target_lat = lat
    target_long = long

def update_lidar_data(data: LaserScan) -> None:
    """
    Update the stored lidar navigation readings

    @param data: Lidar data received
    """
    global lidar_data
    lidar_data = data

def run_navigation() -> bool:
    """
    Drive the rover towards the target angle

    @param delta_heading: Relative value of target from robot from -180 to 180. Starting from bottom of robot (Counter-Clockwise)
    @param result: Angle to drive to based on target angle, current heading, and obstacles. Value given as a sector angle with right of robot being 0 (Counterclockwise).
    @param msg: Get the DriveTrainCmd(motor values) relating to the heading of the robot and the resulting best navigation angle
    """
    if is_active:
        ## Gets best possible angle, considering obstacles
        delta_heading = angle_diff(target_angle, cur_heading)
        # rospy.logerr(delta_heading)
        result = get_navigation_angle(
            ((((90 + delta_heading) % 360) + 360) % 360)
            / math.degrees(
                lidar_data.angle_increment
            ),  # sector angle, #Changes delta_heading to be based on the right of the robot as 0 (Counter-clockwise), and go in increments of sector angle.
            LIDAR_THRESH_DISTANCE,
            lidar_data,
            smoothing_constant,
        )

        # Set the bounds of the speed multiplier
        speed_factor = 0.3
        speed_factor = 0 if speed_factor < 0 else speed_factor
        speed_factor = 1 if speed_factor > 1 else speed_factor
        # Get the DriveTrainCmd relating to the heading of the robot and the resulting best navigation angle
        # Reason we do 90 - result is to get a value where 0 is up, + is clockwise, and - is counterclockwise
        msg = angle_calc.piecewise_linear(angle_diff(90, result), 0)

        #rospy.logerr(result)
        # rospy.loginfo(f"left drive value: {msg.left_value}, right drive value: {msg.right_value}")
        #        t += 2
        #        if t > 90: # t for debugging purposes
        #            t = -90
        # Scale the resultant DriveTrainCmd by the speed multiplier
        msg.left_value *= speed_factor  # Right value was inverted, -1 "fixes"
        msg.right_value *= speed_factor
        # Publish the DriveTrainCmd to the topic
        rospy.loginfo("Heading Value: " + str(cur_heading))
        rospy.loginfo("Target Value: " + str(target_angle))
        drive_pub.publish(msg)

    imu = AngleCalculations(current_lat, current_long,
                            target_lat, target_long)
    if imu.get_distance() < NAV_THRESH_DISTANCE:
        return True
    else:
        return False

# If this file was executed...
if __name__ == "__main__":
    try:
        # Initialize the running environment for this program
        initialize()
        # Spin RosPy to the next update cycle
        rospy.spin()
        

    # Ignore ROS Interrupt Exceptions
    except rospy.ROSInterruptException:
        pass

    # If there is some other error (i.e. ROS Termination)
    finally:
        # Stop the drive motors before shutting down
        print("Stopping motors")
        msg_stop = DriveTrainCmd()
        msg_stop.left_value = 0
        msg_stop.right_value = 0
        drive_pub.publish(msg_stop)
        time.sleep(0.1)

## @}
## @}
