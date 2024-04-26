#!/usr/bin/env python3

"""@file
@defgroup wr_logic_search
@{
@defgroup wr_logic_search_search_action_server Search Action Server
@brief Action server for the rover search pattern
@details Tells the rover to travel to specific coordinates that follow the square 
spiral search pattern. Keeps track of time to know when a coordinate should be 
skipped due to large, unavoidable obstacles. At each coordinate, the camera scans 
the surrounding area, searching for the target object of the current round.
@{
"""

import rospy
import actionlib
from wr_logic_search.msg import SearchStateAction
from wr_logic_longrange.nodes import obstacle_avoidance
import coord_calculations
import travel_timer
# import coord_sub
# import camera_sub 

class SearchActionServer(object):
    def __init__(self, name) -> None:
        """
        Initializes the action server

        @param name (String): Name of the action server
        """

        self._action_name = name
        obstacle_avoidance.initialize()
        # coord_sub.initialize()
        # camera_sub.initialize()
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            SearchStateAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self._as.start()

    def execute_callback(self): 
        """
        Executes the long range obstacle avoidance code, and triggers the 
        corresponding state machine event depending on the result of the 
        navigation.

        @param goal (SearchGoal): Goal for the navigation segment, which contains 
        the GPS coordinates of the target.
        """
        distance = 4
        num_vertices = 22

        # start_lat, start_long = coord_sub.get_coord_result()

        # TODO: During testing, hard-code some coordinates.
        hard_coded_lat = 0
        hard_coded_long = 0

        # coords = coord_calculations.get_coords(
        #     start_lat, start_long, distance, num_vertices)
        coords = coord_calculations.get_coords(
            hard_coded_lat, hard_coded_long, distance, num_vertices)
        SEARCH_TIMEOUT_TIME = travel_timer.calc_state_time() # default = 20 meters

        i = 0
        state_time = rospy.get_rostime()
        point_time = rospy.get_rostime()
        while (
            rospy.get_rostime() - state_time < SEARCH_TIMEOUT_TIME
            and rospy.get_rostime() - point_time < travel_timer.calc_point_time(
                coords[i]['distance'])
            and not rospy.is_shutdown()
            and i < num_vertices
        ):
            if obstacle_avoidance.update_target(
                coords[i]['lat'], coords[i]['long']) == False:
                # return self._as.set_succeeded()
                return self._as.set_aborted()
            
            # if camera_sub.get_detection_result():
            #     break

            i += 1
            point_time = rospy.get_rostime()

if __name__ == "__main__":
    try:
        rospy.init_node("search_action_server")
        server = SearchActionServer("SearchActionServer")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

## @}
## @}