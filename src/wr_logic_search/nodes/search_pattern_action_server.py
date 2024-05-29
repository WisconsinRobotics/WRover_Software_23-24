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

from actionlib import SimpleActionServer, SimpleActionClient, GoalStatus

import wr_logic_search.coord_calculations as coord_calculations
import wr_logic_search.travel_timer as travel_timer

from wr_logic_longrange.msg import LongRangeAction, LongRangeGoal
from wr_logic_search.msg import (
    SearchStateAction,
    SearchStateGoal,
    SearchStateResult,
    SpinAction,
    SpinGoal,
)


class SearchActionServer:
    def __init__(self, name) -> None:
        """
        Initializes the action server

        @param name (String): Name of the action server
        """
        self.long_range_client = SimpleActionClient(
            "LongRangeActionServer", LongRangeAction
        )

        self.spin_client = SimpleActionClient("SpinActionServer", SpinAction)

        self._action_name = name
        self._as = SimpleActionServer(
            self._action_name,
            SearchStateAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self._as.start()

    def execute_callback(self, goal: SearchStateGoal):
        """
        Executes the long range obstacle avoidance code, and triggers the
        corresponding state machine event depending on the result of the
        navigation.

        @param goal (SearchGoal): Goal for the navigation segment, which contains
        the GPS coordinates of the target.
        """
        distance = 4  # in meters, the shortest unit of movement
        num_vertices = 22
        object_detected = False

        # get list of coordinates
        coords = coord_calculations.get_coords(
            goal.initial_lat, goal.initial_long, distance, num_vertices
        )

        # NOTE: in the future, set timeouts using the action client, not the server
        # in seconds, get max time for a single state
        # SEARCH_TIMEOUT_TIME = travel_timer.calc_state_time()

        self.long_range_client.wait_for_server()

        i = 0
        # start timer for the state and the first coordinate
        # state_time = rospy.get_rostime()
        # point_time = rospy.get_rostime()
        while not rospy.is_shutdown() and i < num_vertices:
            # TODO figure out timeout
            timeout = travel_timer.calc_point_time(coords[i]["distance"]) + 15
            self.long_range_client.send_goal(
                LongRangeGoal(
                    target_lat=coords[i]["lat"], target_long=coords[i]["long"]
                )
            )
            self.long_range_client.wait_for_result(rospy.Duration(timeout))
            # self.long_range_client.wait_for_result()
            self.long_range_client.cancel_goal()

            if self.long_range_client.get_state() == GoalStatus.SUCCEEDED:
                # TODO define timeout for spin
                # spin to search for vision target
                self.spin_client.send_goal(SpinGoal(
                    target_id=goal.target_id
                ))
                self.spin_client.wait_for_result(rospy.Duration(60))
                object_detected = self.spin_client.get_state() == GoalStatus.SUCCEEDED
                if object_detected:
                    break

            i += 1  # next coordinate

        # Set result
        if object_detected:
            self._as.set_succeeded(
                SearchStateResult(
                    final_lat=coords[i]["lat"], final_long=coords[i]["long"]
                )
            )
        else:
            self._as.set_aborted(SearchStateResult(0, 0))


if __name__ == "__main__":
    try:
        rospy.init_node("search_action_server")
        server = SearchActionServer("SearchActionServer")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

## @}
## @}
