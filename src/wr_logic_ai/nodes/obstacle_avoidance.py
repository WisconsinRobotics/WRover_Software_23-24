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
from visualization_msgs.msg import Marker
from finder import get_navigation_angle
from angle_calculations import AngleCalculations
import angle_to_drive_methods as angle_calc
from sensor_msgs.msg import LaserScan
from wr_hsi_sensing.msg import CoordinateMsg
from wr_drive_msgs.msg import DriveTrainCmd
from std_msgs.msg import Float64
import testing_rviz

# Navigation parameters
# distance before obstacle avoidance logics is triggered (in meters)
LIDAR_THRESH_DISTANCE = 5
# distance before rover believes it has reached the target (in meters)
NAV_THRESH_DISTANCE = 0.5

# initialize target angle to move forward
# current location
current_lat = 0
current_long = 0
cur_heading = 0

# target location
target_angle = 0
target_sector = 0

# used for obtaining navigation angle and valleys
smoothing_constant = 3

# global speed factor updated through navigation
speed_factor = 0

# Start the tasks managed to drive autonomously
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
    global heading_pub
    global frameCount
    global heading_msg
    global actual_heading_msg
    global actual_heading_pub
    global delta_heading_pub
    global delta_heading_msg
    global marker
    global marker_pub
    global marker_circle
    global marker_circle_pub
    global marker_flag
    global marker_flag_pub
    global laser_adjuster_pub
    global wRover
    global wRover_pub
    global rviz_sim_delta_heading_pub
    global rviz_sim_result_pub
    global rviz_sim_cur_heading_pub

    # Publisher
    drive_pub = rospy.Publisher(
        rospy.get_param("~motor_speeds"), DriveTrainCmd, queue_size=1
    )

    # Subscribe to gps coordinate data
    rospy.Subscriber("/gps_coord_data", CoordinateMsg, update_gps_coord)

    # Subscribe to heading data
    rospy.Subscriber("/heading_data", Float64, update_heading)

    # Subscribe to lidar data
    rospy.Subscriber('/scan', LaserScan, update_navigation)


# update current position based on gps coordinates
def update_gps_coord(msg: CoordinateMsg) -> None:
    global current_lat
    global current_long
    current_lat = msg.latitude
    current_long = msg.longitude


# extends from 0 to 360 degrees, heading is shifted based on east (90 is north -> 0)
def update_heading(msg: Float64) -> None:
    """
    Shifted from compass coordinates to math coordinates.

    @param msg (Float64): Heading value received as value from 0 to 360. North is 0 clockwise.
    @param cur_heading East is 0. (Counterclockwise). Extected as a value from 0 to 360.
    """
    global cur_heading
    cur_heading = (90 - msg.data) % 360  # Shifting to East
    # rviz_sim_cur_heading_pub.publish(cur_heading)


# calculates difference of angles from -180 to 180 degrees
def angle_diff(heading1: float, heading2: float) -> float:
    """std::this_thread::sleep_for(std::chrono::milliseconds(100));
    Returns relative angle difference from heading of robot to heading of target

    @param heading1 (float): Value of target relative to East (Counter-clockwise)
    @param heading2 (float): Value of heading relative to East (Counter-clockwise)
    @return float: Value from -180 to 180. Starting from bottom of robot (Counter-Clockwise)
    """
    diff = (heading1 - heading2 + 360) % 360
    return (diff + 180) % 360 - 180


# Calculate current heading and the planar target angle
# TODO: this should now be part of a action server callback function


# update angle from rover to target based on new current position
def update_target(target_lat, target_long) -> bool:
    """
    Updates target_angle with values from IMU which use gps coords

    @param target_angle: Gives target angle relative to East (counter-clockwise)
    @param imu An object that creates a planar target angle relative to east, accounting for curvature of Earth
    @return bool: If we have arrived to the destination, based on our current and target coords
    """
    global target_angle

    # Construct the planar target angle relative to east, accounting for curvature
    imu = AngleCalculations(current_lat, current_long,
                            target_lat, target_long)

    target_angle = imu.get_angle() % 360

    # check if we are close to the target
    if imu.get_distance() < NAV_THRESH_DISTANCE:
        return True
    else:
        return False


# Update the robot's navigation and drive it towards the target angle based on our best valley
def update_navigation(data: LaserScan) -> None:
    """
    Update the robot's navigation and drive it towards the target angle

    @param delta_heading: Relative value of target from robot from -180 to 180. Starting from bottom of robot (Counter-Clockwise)
    @param result: Angle to drive to based on target angle, current heading, and obstacles. Value given as a sector angle with right of robot being 0 (Counterclockwise).
    @param msg: Get the DriveTrainCmd(motor values) relating to the heading of the robot and the resulting best navigation angle
    @param data: Lidar data received
    """

    global frameCount

    # rospy.loginfo(f"target angle: {target_angle}, current heading: {cur_heading}")
    # calculates average distance 
    data_avg = sum(cur_range for cur_range in data.ranges) / len(data.ranges)

    #print("Data Avg: " + str(data_avg))
    # TODO: data threshold might depend of lidar model, double check
    # Change if units/lidar changes
    # data_avg is above 0.5 almost always, but result stays the same (?)
    # TODO (@bennowotny ): This depended on data_avg, why?
    if True:
        ## Gets best possible angle, considering obstacles
        delta_heading = angle_diff(target_angle, cur_heading)
        
        result = get_navigation_angle(
            ((((90 + delta_heading) % 360) + 360) % 360)
            / math.degrees(
                data.angle_increment
            ),  # sector angle, #Changes delta_heading to be based on the right of the robot as 0 (Counter-clockwise), and go in increments of sector angle.
            LIDAR_THRESH_DISTANCE,
            data,
            smoothing_constant,
        )
        
        # rospy.loginfo(f"raw heading: {result}")

        # Set the bounds of the speed multiplier by clamping it
        speed_factor = 0.3
        if(speed_factor < 0):
            speed_factor = 0
        elif(speed_factor > 1):
            speed_factor = 1

        # Get the DriveTrainCmd relating to the heading of the robot and the resulting best navigation angle
        # Reason we do 90 - result is to get a value where 0 is up, + is clockwise, and - is counterclockwise
        msg = angle_calc.logistic(angle_diff(90, result), 0)

        # rospy.loginfo(f"left drive value: {msg.left_value}, right drive value: {msg.right_value}")
        # Scale the resultant DriveTrainCmd by the speed multiplier
        msg.left_value *= speed_factor 
        msg.right_value *= speed_factor
        
        if msg.left_value > msg.right_value:
            valueToTurn = "Turn right"
        elif msg.left_value < msg.right_value:
            valueToTurn = "Turn left"
        else:
            valueToTurn = "Stay straight"
            
        # Publish the DriveTrainCmd to the topic
        rospy.loginfo("Drive to: " + str(valueToTurn))
        #rospy.loginfo("Target Value: " + str(target_angle))
        drive_pub.publish(msg)

        # TESTING
        testing_rviz.update_navigation_rviz_sim(delta_heading, result, cur_heading)
        


# If this file was executed...
if __name__ == "__main__":
    try:
        # Initialize the running environment for this program
        initialize()
        testing_rviz.initialize()
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
