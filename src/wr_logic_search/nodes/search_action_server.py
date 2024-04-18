#!/usr/bin/env python3

"""@file
@defgroup wr_logic_search
@{
@defgroup wr_logic_search_search_action_server Search Action Server
@brief Tells the rover travel to the target coordinates and calls obstacle avoidance to make sure the rover can actually reach these coordinates.
@details Enters a while loop that continually calls obstacle avoidance to update the target to the next coordinate in the list. 
For each coordinate, if the time between the starting time and the current time go above the estimated travel time for that
specific coordinate, the coordinate is assumed to be unreachable, and the rover will skip it and move on to the next coordinate.
@{
"""

import rospy
import actionlib
from wr_logic_search.msg import SearchStateAction, SearchStateGoal
from wr_logic_longrange.nodes import obstacle_avoidance
import coord_calculations
import travel_timer
# from wr_logic_search.srv import SearchPatternService
# from camera_sub import CameraSub

class SearchActionServer(object):
    def __init__(self, name) -> None:
        """
        Initializes the action server

        @param name (String): Name of the action server
        """

        self._action_name = name
        obstacle_avoidance.initialize()
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            SearchStateAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self._as.start()

    def execute_callback(self, goal: SearchStateGoal):
        """
        Executes the long range obstacle avoidance code, and triggers the corresponding state machine event
        depending on the result of the navigation

        @param goal (SearchGoal): Goal for the navigation segment, which contains the GPS coordinates
        of the target
        """
        distance = 4
        num_vertices = 22

        # TODO: During testing, hard code some coordinates.
        hard_coded_lat = 0
        hard_coded_long = 0

        # coords = coord_calculations.get_coords(goal.starting_lat, goal.starting_long, distance, num_vertices)
        coords = coord_calculations.get_coords(hard_coded_lat, hard_coded_long, distance, num_vertices)
        SEARCH_TIMEOUT_TIME = travel_timer.calc_state_time() # default = 20 meters

        i = 0
        state_time = rospy.get_rostime()
        while (
            rospy.get_rostime() - state_time < SEARCH_TIMEOUT_TIME
            and not rospy.is_shutdown()
            and i < num_vertices
        ):
            # if (i != 0): # skip starting point because rover is already there
            point_time = rospy.get_rostime()
            while (
                rospy.get_rostime() - point_time < travel_timer.calc_point_time(coords[i]['distance'])
                and not rospy.is_shutdown()
            ):
                if obstacle_avoidance.update_target(coords[i]['lat'], coords[i]['long']):
                    # return self._as.set_succeeded()
                    break # successful?
                # TODO: else?????
                    
                # camera_sub = CameraSub()
                # if camera_sub.get_detection_result:
                #     break

            i += 1
            # return self._as.set_aborted()
            
            # # Camera Service - still not sure about integrating the camera with the state machine
            # camera_service = rospy.ServiceProxy('search_pattern_service', SearchPatternService)
            # camera_service.wait_for_service('search_pattern_service')

            # if camera_service:
            #     break # what should be done when the target object is found? how should we enter ShortRange?

if __name__ == "__main__":
    try:
        rospy.init_node("search_action_server")
        server = SearchActionServer("SearchActionServer")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

## @}
## @}