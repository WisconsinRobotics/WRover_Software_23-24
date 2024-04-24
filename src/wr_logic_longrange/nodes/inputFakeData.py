#!/usr/bin/env python3
from numbers import Integral
import random

from numpy import integer
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from wr_hsi_sensing.msg import CoordinateMsg

from wr_drive_msgs.msg import DriveTrainCmd
from geometry_msgs.msg import PoseStamped
from finder import offset_lidar_data
from copy import deepcopy

import math

t=100
#dist = 7
RATE = 10
ROBOT_WIDTH = 1.06 #In meters

#coord[x,y]
class Obstacle:
    '''
    Sends a row of random points to act as walls/obstacles for lidar testing
    '''
    def __init__(self, coord1, coord2):
        self.coord1 = coord1
        self.coord2 = coord2
    def moveRight(self, x):
        self.coord1[0] = self.coord1[0] + x 
        self.coord2[0] = self.coord2[0] + x 
    def moveUp(self, y):
        self.coord1[1] = self.coord1[1] + y
        self.coord2[1] = self.coord2[1] + y
    def getAngle1(self) -> int:
        return int(pointsToAngle(self.coord1[0], self.coord1[1]))
    def getAngle2(self) -> int:
        return int(pointsToAngle(self.coord2[0], self.coord2[1]))
    def setObstacleFront(self):
        self.coord1[0] = 3
        self.coord2[0] = -2
    def testAngle(self):
        self.coord1[0] = random.randint(-5, 5)
        self.coord1[1] = random.randint(5, 10)
        self.coord2[0] = random.randint(-5, 5)
        self.coord2[1] = random.randint(5, 10)
        #rospy.logerr("1:" + str(self.coord1) + "2:" + str(self.coord2))
            
        #Switch which coord is one and which is two as algorithm works when coord 1 is the first one scanning from right to left
        if(self.getAngle1() > self.getAngle2()):
            self.switchCoords()
    def getSlope(self):
        if(self.coord2[0] - self.coord1[0] == 0):
            return 99999
        else:
            return (self.coord2[1] - self.coord1[1])/(self.coord2[0] - self.coord1[0])
    def setCoord1(self, x,y):
        self.coord1[0] = x
        self.coord1[1] = y        
    def setCoord2(self, x,y):
        self.coord2[0] = x
        self.coord2[1] = y
    def switchCoords(self):
        placeholder = self.coord1
        self.coord1 = self.coord2
        self.coord2 = placeholder

obstacle = Obstacle([1,5],[-1,5])
#obstacle = Obstacle([2,6],[3,9]) TODO:Test this error case

def get_laser_ranges(t=0):
    inputData = []
    
    angle1 = obstacle.getAngle1()
    angle2 = obstacle.getAngle2()

    if angle1 > 180:
        angle1 = 0
    
    if(obstacle.getAngle1() > obstacle.getAngle2() and obstacle.getAngle1() < 180):
        obstacle.switchCoords()

    m_slope = obstacle.getSlope()
    for t in range(180):
        if(t >= angle1 and t <= angle2):
            if(t != 0):
                #Using point-slope formula and knowing the equation for the line between the two points and the line made by the angle,
                #you can find the intersection of those lines and plot that point in rviz

                #y=(y_1+x_1*m_slope)/(1-(m_slope/m_angle))
                #x=y/m_angle
                m_angle = math.tan(math.radians(t))

                y = (obstacle.coord1[1] - obstacle.coord1[0]*m_slope)/(1-(m_slope/m_angle))
                x = y/m_angle

                inputData.append(cartesianToRadian(x,y))
                
        else:
            inputData.append(10)
        # if(t >= angle1 and t <= angle1 + ((angle2 - angle1) / 2)):
        #     if(t != 0):
        #         #Using point-slope formula and knowing the equation for the line between the two points and the line made by the angle,
        #         #you can find the intersection of those lines and plot that point in rviz

        #         #y=(y_1+x_1*m_slope)/(1-(m_slope/m_angle))
        #         #x=y/m_angle
        #         m_angle = math.tan(math.radians(t))

        #         y = (obstacle.coord1[1] - obstacle.coord1[0]*m_slope * 2)/(1-(m_slope * 2/m_angle))
        #         x = y/m_angle

        #         inputData.append(cartesianToRadian(x,y))
                
        # elif (t >= angle1 and t <= angle2):
        #     if(t != 0):
        #         #Using point-slope formula and knowing the equation for the line between the two points and the line made by the angle,
        #         #you can find the intersection of those lines and plot that point in rviz

        #         #y=(y_1+x_1*m_slope)/(1-(m_slope/m_angle))
        #         #x=y/m_angle
        #         m_angle = math.tan(math.radians(t))

        #         y = (obstacle.coord1[1] - obstacle.coord1[0]*m_slope)/(1-(m_slope/m_angle))
        #         x = y/m_angle

        #         inputData.append(cartesianToRadian(x,y))
        # else:
        #     inputData.append(10)

    for i in range(180):
        inputData.append(.3)

    return inputData


def pointsToAngle(x,y) -> float:
    return math.degrees(math.atan2(y,x)) % 360 #the Mod 360 turns -160 to 200

def cartesianToRadian(x,y) -> float:
    return math.sqrt(x*x+y*y)

def run_mock_data() -> None:
    global mock_heading
    global laser_adjust

    distanceData = rospy.Publisher('/scan', LaserScan, queue_size=10)
    mock_heading_pub = rospy.Publisher('/heading_data', Float64, queue_size=10)
    mock_gps_pub = rospy.Publisher(
        '/gps_coord_data', CoordinateMsg, queue_size=10)
    zero_pub = rospy.Publisher('/debug_zero', PoseStamped, queue_size=10)
    
    zero_msg = PoseStamped()
    zero_msg.pose.position.x = 0
    zero_msg.pose.position.y = 0
    zero_msg.pose.position.z = 0
    zero_msg.pose.orientation.x = 0
    zero_msg.pose.orientation.y = 0
    zero_msg.pose.orientation.z = 0
    zero_msg.pose.orientation.w = 1
    zero_msg.header.frame_id = "laser"
    frameCount = 0

    # Testing publishers and subscribers
    rospy.Subscriber('/control/drive_system/cmd', DriveTrainCmd, updateHeading)

    laser = LaserScan()
    laser.angle_max = 2 * math.pi
    laser.angle_increment = math.pi / 180
    laser.time_increment = 0
    laser.scan_time = 1
    laser.range_min = 0
    laser.range_max = 150
    laser.ranges = get_laser_ranges(0)
    laser.header.frame_id = "laser"
    laser.intensities = []

    mock_heading = 0
    laser_adjust = 0
    mock_gps = CoordinateMsg()
    mock_gps.latitude = 0
    mock_gps.longitude = 0

    rospy.loginfo("sent fake nav data")
    
    sleeper = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        laser.ranges = get_laser_ranges(t)
        distanceData.publish(laser)

        zero_msg.header.seq = frameCount
        zero_msg.header.stamp = rospy.get_rostime()
        frameCount += 1

        mock_heading_pub.publish(mock_heading)
        mock_gps_pub.publish(mock_gps)
        zero_pub.publish(zero_msg)
        sleeper.sleep()


def updateHeading(data) -> None:
    global mock_heading
    global t
    #global dist
    #global height
    
    #Move obstacle up and down
    # distance = velocity * time
    # distance = amount of movement of obstacle
    # velocity = average of wheel speeds * max velocity
    # time = (1/heartz) [Heartz is rospy.sleep]
    maxVelocity = .8 ### in meters per second
    obstacle.moveUp((-(data.left_value + data.right_value))*maxVelocity*(1/RATE))


    #amount rotated will depend on our angular velocity 
    #amount rotated = angular velocity * time
    #amount rotated = laser_adjust
    #------------------
    #w(angular velocity) = v/r
    #angular velocity = (velocity of left value + velocity of right value) / radius of robot
    #angular velocity = (data.left_value*maxVelocity + data.right_value*maxVelocity) / radius of robot
    #angular velocity will return in radians per second

    #-----------------
    laser_adjust = (data.left_value*maxVelocity - data.right_value*maxVelocity) / (ROBOT_WIDTH/2)
    laser_adjust = laser_adjust*(1/RATE)
    #rospy.logerr(laser_adjust)
    
    mock_heading = (
        mock_heading + math.degrees(laser_adjust))
    
    #rospy.logerr(mock_heading)
    
    #Rotate obstacle:
    #Refer to this graph for the equation https://www.desmos.com/calculator/anccue0csk
    
    angle_adjust = -laser_adjust

    obstacle.setCoord1(obstacle.coord1[0]*math.cos((angle_adjust)) + obstacle.coord1[1]*math.sin((angle_adjust)),
                         obstacle.coord1[1]*math.cos((angle_adjust)) - obstacle.coord1[0]*math.sin((angle_adjust)))

    obstacle.setCoord2(obstacle.coord2[0]*math.cos((angle_adjust)) + obstacle.coord2[1]*math.sin((angle_adjust)),
                         obstacle.coord2[1]*math.cos((angle_adjust)) - obstacle.coord2[0]*math.sin((angle_adjust)))

    # rospy.logerr(str(obstacle.coord2[0]) + " " + str(obstacle.coord2[1]))

    #******* Reset obstacle when behind us *************#
    if(obstacle.coord1[1] < .1 and obstacle.coord2[1] <.1):
        obstacle.testAngle()    


if __name__ == "__main__":
    rospy.init_node("publish_fake_data", anonymous=False)

    if rospy.get_param("/long_range_action_server/wrover_hw") == "MOCK":
        # Run fake data
        rospy.loginfo("Running fake data")
        run_mock_data()

    rospy.spin()
