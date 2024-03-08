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

    # angle1 = int(math.asin(height/(cartesianToRadian(xDist1,height))))

    # angle2 = int(math.asin(height/(cartesianToRadian(xDist2,height))))
    angle1 = obstacle.getAngle1()
    angle2 = obstacle.getAngle2()

    if angle1 > 180:
        angle1 = 0
    
    if(obstacle.getAngle1() > obstacle.getAngle2() and obstacle.getAngle1() < 180):
        obstacle.switchCoords()
        rospy.logerr("SWWWWWWWWWWWWWWIIIIIIITCCCCCCCCCHHHHH")

    # rospy.logerr("-------------")
    # rospy.logerr(angle1)
    # rospy.logerr(angle2)
    # rospy.logerr("-------------")
    #rospy.logerr(angle1)

   # rospy.logerr("\n1:" + str(obstacle.coord1) + "\n2:" + str(obstacle.coord2))


    # modifierx = 0
    # modifiery = 0
    # post_modifierx = 0
    # post_modifiery = 0

    m_slope = obstacle.getSlope()

    # post_modifierx=((obstacle.coord1[0] + post_modifierx)*math.tan(math.radians(angle1))) - obstacle.coord1[1]
    # post_modifiery=((obstacle.coord1[1] + post_modifiery)/math.tan(math.radians(angle1))) - obstacle.coord1[0]
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
                # rospy.logerr("x" + " " + str(t) + " " + str(x))
                # rospy.logerr("y" + " " + str(t) + " " + str(y))

                inputData.append(cartesianToRadian(x,y))

                # rospy.logerr(angle1)
                # modifiery = ((obstacle.coord1[0] + post_modifierx)*math.tan(math.radians(t))) - obstacle.coord1[1]
                # rospy.logerr("y" + " " + str(t) + " " + str(modifiery))
                # modifierx = ((obstacle.coord1[1] + post_modifiery)/math.tan(math.radians(t))) - obstacle.coord1[0] # get x distance x = height/tan(angle)
                # #rospy.logerr("x" + " " + str(t) + " " + str(modifierx))
                # post_modifiery= modifiery
                # post_modifierx = modifierx

                

                # rospy.logerr(slope)
                # if(slope == 0):
                #     inputData.append(cartesianToRadian( ((y_obs-obstacle.coord1[1])/slope)+obstacle.coord1[0] , slope*(x_obs-obstacle.coord1[0])-obstacle.coord1[1]))
                # else:
                #     inputData.append(cartesianToRadian( ((y_obs-obstacle.coord1[1])/slope)+obstacle.coord1[0] , slope*(x_obs-obstacle.coord1[0])-obstacle.coord1[1]))
                
            # else:
            #     modifierx = 0
            #     modifiery = 0
            # inputData.append(cartesianToRadian(obstacle.coord1[0]+modifierx, obstacle.coord1[1] +modifiery))

            #rospy.logerr(str(t) + " " +str(cartesianToRadian(obstacle.coord1[0]+modifier, obstacle.coord1[1])))
        else:
            inputData.append(10)

    for i in range(180):
        inputData.append(.3)

    # for t in range(180):
    #     distance = 10
    #     if t -90 <= t and t <= t and t != 0:
    #         distance = dist
    #     inputData.append(distance)
    # for i in range(180):
    #     inputData.append(.3)
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
    #rospy.Subscriber('/laser_adjuster', Float64, updateLaserAdjuster)

    laser = LaserScan()
    # vara.intensities

    # laser.angle_min = 0.
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

    print("sent fake nav data")
    
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
# def updateLaserAdjuster(data):
#     global laser_adjust
#     laser_adjust = -data.data/30

def updateHeading(data) -> None:
    global mock_heading
    global t
    #global dist
    #global height
    
    #t+= ((data.left_value - data.right_value)*3)*(4)

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
    #w = v/r
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

    #******* Reset obstacle *************#
    if(obstacle.coord1[1] < .1 and obstacle.coord2[1] <.1):
        obstacle.testAngle()    
        # rospy.logerr(obstacle.coord1[1])
        # rospy.logerr(obstacle.coord2[1])


    # dist = dist - .1*((data.left_value + data.right_value)/2)
    # #t+=2
    # t%=360
    # if(dist <= 0.1 or (dist < 2.8 and data.left_value >= .298 and data.right_value >= .298)):
    #     t=100+(random.randint(0,70))
    #     dist=7
   


def display_data(data) -> None:
    rviz_data = deepcopy(data)

    # rospy.logerr((data.ranges[287]))
    # print(data.ranges[287])

    # rospy.loginfo("Front " + str(data.ranges[0 * 287]))
    # rospy.loginfo("Back " + str(data.ranges[2 * 287]))


    rviz_data.ranges = offset_lidar_data(
        rviz_data.ranges, math.degrees(rviz_data.angle_increment), True)
    #scan_rviz_pub = rospy.Publisher('/scan_rviz', LaserScan, queue_size=10)
    #scan_rviz_pub.publish(rviz_data)


def run_real_data() -> None:
    rospy.Subscriber("/scan", LaserScan, display_data)


if __name__ == "__main__":
    rospy.init_node("publish_fake_data", anonymous=False)

    # rospy.loginfo(rospy.get_param('WROVER_HW'))

    rospy.loginfo(rospy.get_param("/long_range_action_server/wrover_hw"))

    # params_list = rospy.get_param_names()

    # for param in params_list:
    #     rospy.logerr("parameter: " + str(param))

    if rospy.get_param("/long_range_action_server/wrover_hw") == "MOCK":
        # Run fake data
        rospy.loginfo("Running fake data")
        run_mock_data()
    else:
        # Run real data
        run_real_data()
    
    rospy.spin()
