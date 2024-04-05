#!/usr/bin/env python

"""@file
@defgroup wr_logic_search
@{
@defgroup wr_logic_search_search_pattern_server Search Pattern Server
@brief The server for the camera that scans the rover's surroundings for the target object.
@details Currently, this is a simulated server where pressing 'w' is a success and 
pressing 'l' is a fail. This was originally for testing the search pattern with FreeFall
and will probably be changed in the future as we learn how to integrate the camera into
the state machine software. 
@{
"""

import rospy
from wr_logic_search.srv import SearchPatternService, SearchPatternServiceRequest, SearchPatternServiceResponse
import keyboard  # I guess we don't need this anymore since we're not using FreeFall

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

## @}
## @}