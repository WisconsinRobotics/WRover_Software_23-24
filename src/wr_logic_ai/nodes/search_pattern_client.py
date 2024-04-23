import rospy
from wr_logic_ai.srv import SearchPatternService, SearchPatternServiceRequest, SearchPatternServiceResponse

def main():
    rospy.init_node("search_pattern_client")
    search_pattern_service = rospy.ServiceProxy('search_pattern_service', SearchPatternService)
    rate = rospy.Rate(10)
    rospy.wait_for_service('object_detection')

    while not rospy.is_shutdown():
        try:
            response:SearchPatternServiceResponse = search_pattern_service()
            return response.output
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

        rate.sleep()

if __name__ == '__main__':
    main()