#!/usr/bin/env python3

"""@file
@defgroup wr_logic_ai_longrange_ai
@{
@defgroup wr_logic_ai_longrange_ai_testing_rviz Testing Rviz
@brief 
@details 
@{
"""

import math
import rospy
import time
from visualization_msgs.msg import Marker
from finder import get_navigation_angle
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

# Navigation parameters
# distance before obstacle avoidance logics is triggered (in meters)
LIDAR_THRESH_DISTANCE = 5
# distance before rover believes it has reached the target (in meters)
NAV_THRESH_DISTANCE = 0.5

# initialize target angle to move forward
# current location
current_lat = 0
current_long = 0

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
    #global raw_heading_pub
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
    global delta_heading

    # Publish data out to the marker 
    wRover_pub = rospy.Publisher('wRover_marker', Marker, queue_size=10)

    # rospy.Subscriber("/rviz_simulation_heading", CoordinateMsg, update_sim_heading)
    # rospy.Subscriber("/rviz_simulation_result", CoordinateMsg, update_sim_result)
    # rospy.Subscriber("/rviz_simulation_current", CoordinateMsg, update_sim_current)


    # def update_sim_heading(msg: Float64) -> None:
    #     global delta_heading
    #     delta_heading = msg.data

    # def update_sim_result(msg: Float64) -> None:
    #     global result
    #     result = msg.data

    # def update_sim_current(msg: Float64) -> None:
    #     global cur_heading
    #     cur_heading = msg.data


    # TESING
    heading_pub = rospy.Publisher("/debug_heading", PoseStamped, queue_size=1)
    heading_msg = PoseStamped()
    heading_msg.pose.position.x = 0
    heading_msg.pose.position.y = 0
    heading_msg.pose.position.z = 0
    heading_msg.pose.orientation.x = 0
    heading_msg.pose.orientation.y = 0
    heading_msg.header.frame_id = "laser"
    frameCount = 0
    # raw_heading_pub = rospy.Publisher(
    #     '/target_heading_raw', Float64, queue_size=1)
    
     # TESING
    actual_heading_pub = rospy.Publisher('/actual_heading', PoseStamped, queue_size=1)
    actual_heading_msg = PoseStamped()
    actual_heading_msg.pose.position.x = 0
    actual_heading_msg.pose.position.y = 0
    actual_heading_msg.pose.position.z = 0
    actual_heading_msg.pose.orientation.x = 0
    actual_heading_msg.pose.orientation.y = 0
    actual_heading_msg.header.frame_id = "laser"

     # TESING
    delta_heading_pub = rospy.Publisher('/delta_heading', PoseStamped, queue_size=1)
    delta_heading_msg = PoseStamped()
    delta_heading_msg.pose.position.x = 0
    delta_heading_msg.pose.position.y = 0
    delta_heading_msg.pose.position.z = 0
    delta_heading_msg.pose.orientation.x = 0
    delta_heading_msg.pose.orientation.y = 0
    delta_heading_msg.header.frame_id = "laser"
    

    laser_adjuster_pub = rospy.Publisher('/laser_adjuster', Float64, queue_size=1)


    marker_pub = rospy.Publisher("pose_marker", Marker, queue_size=1)
    marker = Marker()
    marker.header.frame_id = "laser"  # Set your frame_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.scale.x = 10.0  # Set the dimensions of the plane
    marker.scale.y = 10.0
    marker.scale.z = 0.01  # Thickness of the plane
    marker.color.a = 0.5
    marker.color.r = .1
    marker.color.g = .6
    marker.color.b = .1
    marker.pose = delta_heading_msg.pose  # Set the position and orientation based on your pose

    marker_circle_pub = rospy.Publisher("pose_circle_marker", Marker, queue_size=1)
    marker_circle = Marker()
    marker_circle.header.frame_id = "laser"  # Set your frame_id
    marker_circle.type = Marker.SPHERE
    marker_circle.action = Marker.ADD
    marker_circle.scale.x = LIDAR_THRESH_DISTANCE*2  # Set the dimensions of the plane
    marker_circle.scale.y = LIDAR_THRESH_DISTANCE*2
    marker_circle.scale.z = 0.01  # Thickness of the plane
    marker_circle.color.a = 0.5
    marker_circle.color.r = .1
    marker_circle.color.g = .3
    marker_circle.color.b = .6
    marker_circle.pose = delta_heading_msg.pose  # Set the position and orientation based on your pose

    wRover = Marker()
    wRover.header.frame_id = "laser"  # Replace with your fixed frame
    wRover.type = Marker.MESH_RESOURCE
    wRover.mesh_resource = "package://wr_logic_ai/meshes/Eclipse_Base_Simple.stl"  # Replace with your 3D object path
    wRover.pose.position.x = 0.0  # Replace with your desired position
    wRover.pose.position.y = 0.0
    wRover.pose.position.z = .11
    wRover.pose.orientation.x = 0.0  # Replace with your desired orientation
    wRover.pose.orientation.y = 0.0
    wRover.pose.orientation.z = 0.0
    wRover.pose.orientation.w = 1.0
    wRover.scale.x = 0.01  # Replace with your desired scale
    wRover.scale.y = 0.01
    wRover.scale.z = 0.01
    wRover.color.a = 1.0
    wRover.color.r = 1.0
    wRover.color.g = 0.0
    wRover.color.b = 0.0
    wRover_pub.publish(wRover)

    marker_flag_pub = rospy.Publisher('flag_marker', Marker, queue_size=1)
    marker_flag = Marker()
    marker_flag.header.frame_id = "laser"  # Replace with your fixed frame
    marker_flag.type = Marker.MESH_RESOURCE
    marker_flag.mesh_resource = "package://wr_logic_ai/meshes/flag.stl"  # Replace with your 3D object path
    marker_flag.pose.position.x = 0.0  # Replace with your desired position
    marker_flag.pose.position.y = 5.0
    marker_flag.pose.position.z = .11
    marker_flag.pose.orientation.x = 0.0  # Replace with your desired orientation
    marker_flag.pose.orientation.y = 0.0
    marker_flag.pose.orientation.z = 0.0
    marker_flag.pose.orientation.w = 1.0
    marker_flag.scale.x = 0.02  # Replace with your desired scale
    marker_flag.scale.y = 0.02
    marker_flag.scale.z = 0.02
    marker_flag.color.a = 1.0
    marker_flag.color.r = 1.0
    marker_flag.color.g = 0.0
    marker_flag.color.b = 0.0
    


# TESTING
def update_navigation_rviz_sim(delta_heading: Float64, result: Float64, cur_heading: Float64) -> None:
    if rospy.get_param("/long_range_action_server/wrover_hw") == "MOCK":
        heading_msg.header.seq = frameCount
        heading_msg.header.stamp = rospy.get_rostime()

        actual_heading_msg.header.seq = frameCount
        actual_heading_msg.header.stamp = rospy.get_rostime()

        delta_heading_msg.header.seq = frameCount
        delta_heading_msg.header.stamp = rospy.get_rostime()

        frameCount += 1
        # negative sign on the pose is hardcoded, may not model how the actual robot will act
        heading_msg.pose.orientation.z = math.sin(math.radians(result) / 2)
        heading_msg.pose.orientation.w = math.cos(math.radians(result) / 2)
        heading_pub.publish(heading_msg)

        # TESTING

        actual_heading_msg.pose.orientation.z = math.sin(math.radians(cur_heading) / 2)
        actual_heading_msg.pose.orientation.w = math.cos(math.radians(cur_heading) / 2)
        actual_heading_pub.publish(actual_heading_msg)

        # TESTING

        delta_heading_msg.pose.orientation.z = math.sin(math.radians(delta_heading + 90) / 2)
        delta_heading_msg.pose.orientation.w = math.cos(math.radians(delta_heading + 90) / 2)
        delta_heading_pub.publish(delta_heading_msg)

        # TESTING

        # Adding the nums is cuz the flag is off center, I'm not sure why??? I downloaded this from a random website :D
        marker_flag.pose.position.x =  5*math.sin(math.radians(delta_heading)) + 0.57
        marker_flag.pose.position.y = -5*math.cos(math.radians(delta_heading)) + .3
        marker_flag_pub.publish(marker_flag)

        laser_adjuster_pub.publish(delta_heading) #Used for inputFakeData

        marker.pose.orientation.z = math.sin(math.radians(delta_heading) / 2)
        marker.pose.orientation.w = math.cos(math.radians(delta_heading) / 2)
        # Set the position and orientation based on delta-heading

        # Publish the Marker message
        marker_pub.publish(marker)

    #Flipped lines to match laser scan from rpLIDAR
    elif rospy.get_param("/long_range_action_server/wrover_hw") == "REAL":

        heading_msg.header.seq = frameCount
        heading_msg.header.stamp = rospy.get_rostime()

        actual_heading_msg.header.seq = frameCount
        actual_heading_msg.header.stamp = rospy.get_rostime()

        delta_heading_msg.header.seq = frameCount
        delta_heading_msg.header.stamp = rospy.get_rostime()

        frameCount += 1
        # negative sign on the pose is hardcoded, may not model how the actual robot will act
        heading_msg.pose.orientation.z = math.sin(math.radians(result+180) / 2)
        heading_msg.pose.orientation.w = math.cos(math.radians(result+180) / 2)
        heading_pub.publish(heading_msg)

        # TESTING

        actual_heading_msg.pose.orientation.z = math.sin(math.radians(-cur_heading) / 2)
        actual_heading_msg.pose.orientation.w = math.cos(math.radians(cur_heading) / 2)
        actual_heading_pub.publish(actual_heading_msg)

        # TESTING

        delta_heading_msg.pose.orientation.z = math.sin(math.radians(delta_heading - 90) / 2)
        delta_heading_msg.pose.orientation.w = math.cos(math.radians(delta_heading + 90) / 2)
        delta_heading_pub.publish(delta_heading_msg)

        # TESTING

        # Adding the nums is cuz the flag is off center, I'm not sure why??? I downloaded this from a random website :D
        marker_flag.pose.position.x =  5*math.sin(math.radians(delta_heading)) + 0.57
        marker_flag.pose.position.y = -5*math.cos(math.radians(delta_heading)) + .3
        marker_flag_pub.publish(marker_flag)

        laser_adjuster_pub.publish(delta_heading) #Used for inputFakeData

        marker.pose.orientation.z = math.sin(math.radians(delta_heading) / 2)
        marker.pose.orientation.w = math.cos(math.radians(delta_heading) / 2)
        # Set the position and orientation based on delta-heading

        # Publish the Marker message
        marker_pub.publish(marker)
    
    if(frameCount < 10):
        wRover_pub.publish(wRover)
        marker_circle_pub.publish(marker_circle)


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
        time.sleep(0.1)

## @}
## @}
