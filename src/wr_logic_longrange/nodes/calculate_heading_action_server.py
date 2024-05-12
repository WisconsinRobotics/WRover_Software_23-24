import rospy
import math
import actionlib
from wr_logic_longrange.msg import InitCompassAction, InitCompassGoal
from wrevolution.msg import SetHeading
from wr_drive_msgs.msg import DriveTrainCmd
from wr_hsi_sensing.msg import CoordinateMsg

LONG_RANGE_TIMEOUT_TIME = rospy.Duration(10)

current_lat = 0
current_long = 0

class InitCompassActionServer(object):
    def __init__(self, name) -> None:
        global drive_pub
        global set_heading_pub
        rospy.loginfo("initing long range action server")
        # Publisher
        drive_pub = rospy.Publisher(
            rospy.get_param("~motor_speeds"), DriveTrainCmd, queue_size=10
        )
        set_heading_pub = rospy.Publisher('set_heading', SetHeading, queue_size=10)

        #Subscribe to GPS Data
        self.subGPS = rospy.Subscriber("/gps_coord_data", CoordinateMsg, update_gps_coord)
        self.rate = 100       
        self.sub_msg = None

        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            InitCompassAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self._as.start()
        
        

    def execute_callback(self, goal: InitCompassGoal):
        
        rate = rospy.Rate(10)
        start_time = rospy.get_rostime()
        drive_msg = DriveTrainCmd(
            left_value=.3,
            right_value=.3,
        )

        #Wait for subscriber to get data
        r = rospy.Rate(self.rate)
        while self.sub_msg == None:
            r.sleep()

        #Get current lat and long before moving
        lat_before = current_lat
        long_before = current_long

        #Driving forward for 10 seconds
        while rospy.get_rostime() - start_time < LONG_RANGE_TIMEOUT_TIME and not rospy.is_shutdown():
            rate.sleep()
            drive_pub.publish(drive_msg)

        heading_msg = SetHeading()
        heading_msg.heading = calculate_angle(long_before, lat_before, current_long, current_lat)
        set_heading_pub.publish(heading_msg)
        
        return self._as.set_succeeded()

# update current position based on gps coordinates
def update_gps_coord(msg: CoordinateMsg) -> None:
    global current_lat
    global current_long
    current_lat = msg.latitude
    current_long = msg.longitude

def calculate_angle(x1, y1, x2, y2):
    """
    Calculate the angle (in degrees) between two sets of coordinates in math coordinates.
    """
    # Calculate the differences in x and y coordinates
    dx = x2 - x1
    dy = y2 - y1
    
    # Calculate the angle using arctan2, convert from radians to degrees
    angle_radians = math.atan2(dy, dx)
    angle_degrees = math.degrees(angle_radians)
    
    # Ensure angle is between 0 and 360 degrees
    if angle_degrees < 0:
        angle_degrees += 360
    
    return angle_degrees

if __name__ == "__main__":
    rospy.init_node("calculate_init_heading")
    initComp = InitCompassActionServer("InitCompass")
    rospy.spin()