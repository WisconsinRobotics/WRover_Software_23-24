#!/usr/bin/env python

import rospy
from wr_logic_ai.srv import SearchPatternService, SearchPatternServiceRequest, SearchPatternServiceResponse
import keyboard  # I guess, we don't need this anymore since we're not using FreeFall

def search_pattern_handler(request: SearchPatternServiceRequest):
    target_found = False

    # simulate the camera detecting the target
    if keyboard.is_pressed('w'):
        target_found = True
        return target_found # return what??
    elif keyboard.is_pressed('l'):
        return target_found # return what??

# initialize the server node
def search_pattern_server():
    rospy.init_node('search_pattern_server')
    s = rospy.Service('search_pattern_service', SearchPatternService, search_pattern_handler)
    rospy.spin()

if __name__ == '__main__':
    search_pattern_server()