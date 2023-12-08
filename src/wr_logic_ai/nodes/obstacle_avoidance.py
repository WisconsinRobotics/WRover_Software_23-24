#!/usr/bin/env python3
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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

# Navigation parameters
# distance before obstacle avoidance logics is triggered (in meters)
LIDAR_THRESH_DISTANCE = 3
# distance before rover believes it has reached the target (in meters)
NAV_THRESH_DISTANCE = 0.5

# initialize target angle to move forward
current_lat = 0
current_long = 0
cur_heading = 0
target_angle = 0
target_sector = 0
smoothing_constant = 3
# Get the speed multiplier of the current runtime for the obstacle_avoidance
# 0.2 is bugged
speed_factor = 0
is_active = False

# Start the tasks managed to drive autonomously


def initialize() -> None:
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

    # Publisher
    drive_pub = rospy.Publisher(rospy.get_param(
        '~motor_speeds'), DriveTrainCmd, queue_size=1)

    # Subscribe to gps coordinate data
    rospy.Subscriber('/gps_coord_data', CoordinateMsg, update_gps_coord)

    # Subscribe to heading data
    rospy.Subscriber('/heading_data', Float64, update_heading)

    # Subscribe to lidar data
    rospy.Subscriber('/scan', LaserScan, update_navigation)

    wRover_pub = rospy.Publisher('wRover_marker', Marker, queue_size=10)


    # TESING
    heading_pub = rospy.Publisher('/debug_heading', PoseStamped, queue_size=1)
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
    


def update_gps_coord(msg: CoordinateMsg) -> None:
    global current_lat
    global current_long
    current_lat = msg.latitude
    current_long = msg.longitude


# extected as 0 to 360 from  North (Clockwise)
def update_heading(msg: Float64) -> None:
    global cur_heading
    cur_heading = (90-msg.data) % 360  # Shifting to East


def angle_diff(heading1: float, heading2: float) -> float:
    diff = (heading1 - heading2 + 360) % 360
    return (diff + 180) % 360 - 180

# Calculate current heading and the planar target angle
# TODO: this should now be part of a action server callback function


def update_target(target_lat, target_long) -> bool:
    global target_angle

    # Construct the planar target angle relative to east, accounting for curvature
    imu = AngleCalculations(current_lat, current_long,
                            target_lat, target_long)
    
    
    target_angle = imu.get_angle() % 360

    if imu.get_distance() < NAV_THRESH_DISTANCE:
        return True
    else:
        return False

# t = 0
# Update the robot's navigation and drive it towards the target angle


def update_navigation(data: LaserScan) -> None:
    global frameCount

    # rospy.loginfo(f"target angle: {target_angle}, current heading: {cur_heading}")
    data_avg = sum(cur_range for cur_range in data.ranges) / len(data.ranges)
    #print("Data Avg: " + str(data_avg))
    # TODO: data threshold might depend of lidar model, double check
    # Change if units/lidar changes
    # data_avg is above 0.5 almost always, but result stays the same (?)
    # TODO (@bennowotny ): This depended on data_avg, why?
    if True:
        # Gets best possible angle, considering obstacles
        delta_heading = angle_diff(target_angle, cur_heading)
        result = get_navigation_angle(
            ((((90 + delta_heading) % 360) + 360) % 360) /
            math.degrees(data.angle_increment),  # sector angle
            LIDAR_THRESH_DISTANCE,
            data,
            smoothing_constant)

        #raw_heading_pub.publish(result)
        # rospy.loginfo(f"raw heading: {result}")

        # Set the bounds of the speed multiplier
        speed_factor = 0.3
        speed_factor = 0 if speed_factor < 0 else speed_factor
        speed_factor = 1 if speed_factor > 1 else speed_factor
        # Get the DriveTrainCmd relating to the heading of the robot and the resulting best navigation angle
        msg = angle_calc.piecewise_linear(angle_diff(90, result), 0)
        # rospy.loginfo(f"left drive value: {msg.left_value}, right drive value: {msg.right_value}")
#        t += 2
#        if t > 90: # t for debugging purposes
#            t = -90
        # Scale the resultant DriveTrainCmd by the speed multiplier
        msg.left_value *= speed_factor  # Right value was inverted, -1 "fixes"
        msg.right_value *= speed_factor
        # Publish the DriveTrainCmd to the topic
        #print("Left Value: " + str(msg.left_value))
        #print("Right Value: " + str(msg.right_value))
        drive_pub.publish(msg)

        # TESTING
        heading_msg.header.seq = frameCount
        heading_msg.header.stamp = rospy.get_rostime()

        actual_heading_msg.header.seq = frameCount
        actual_heading_msg.header.stamp = rospy.get_rostime()

        delta_heading_msg.header.seq = frameCount
        delta_heading_msg.header.stamp = rospy.get_rostime()

        frameCount += 1
        heading_msg.pose.orientation.z = math.sin(math.radians(result) / 2)
        heading_msg.pose.orientation.w = math.cos(math.radians(result) / 2)
        heading_pub.publish(heading_msg)

        # TESTING

        actual_heading_msg.pose.orientation.z = math.sin(math.radians(cur_heading) / 2)
        actual_heading_msg.pose.orientation.w = math.cos(math.radians(cur_heading) / 2)
        actual_heading_pub.publish(actual_heading_msg)

         # TESTING

        delta_heading_msg.pose.orientation.z = math.sin(math.radians(delta_heading+90) / 2)
        delta_heading_msg.pose.orientation.w = math.cos(math.radians(delta_heading+90) / 2)
        delta_heading_pub.publish(delta_heading_msg)

        # TESTING

        # Adding the nums is cuz the flag is off center, I'm not sure why??? I downloaded this from a random website :D
        marker_flag.pose.position.x =  -5*math.sin(math.radians(delta_heading)) + 0.57
        marker_flag.pose.position.y = 5*math.cos(math.radians(delta_heading)) + .3
        marker_flag_pub.publish(marker_flag)

        laser_adjuster_pub.publish(delta_heading) #Used for inputFakeData

        marker.pose.orientation.z = math.sin(math.radians(delta_heading) / 2)
        marker.pose.orientation.w = math.cos(math.radians(delta_heading) / 2)
        # Set the position and orientation based on delta-heading

        # Publish the Marker message
        marker_pub.publish(marker)

        # Publish the circle :D
        #rospy.logerr(rospy.Rate.remaining(0x7f594e3e5f70))

        # Publish the wRover message
        
        if(frameCount < 10):
            wRover_pub.publish(wRover)
            marker_circle_pub.publish(marker_circle)


# If this file was executed...
if __name__ == '__main__':
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
