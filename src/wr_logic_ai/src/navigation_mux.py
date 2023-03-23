import rospy
import topic_tools

long_range_nav = rospy.get_param("long_range_output")

def initialize():
    rospy.Subscriber('/distance_to_target', float, check_long_range_status)

def check_long_range_status(data):
    if data < 0.5:
        rospy.wait_for_service("long_to_short_range")
        try:
            long_to_short_range = rospy.ServiceProxy("long_to_short_range", topic_tools.MuxSelect)
            response = long_to_short_range("lmao xd")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == "__main__":
    initialize()
    rospy.spin()