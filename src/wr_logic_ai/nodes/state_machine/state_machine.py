#!/usr/bin/env python3

import rospy
from smach import StateMachine, State
from smach_ros import SimpleActionState
from std_srvs.srv import Empty

from wr_logic_longrange.msg import LongRangeAction, LongRangeGoal
from wr_logic_longrange.msg import InitCompassAction, InitCompassGoal
from wr_logic_search.msg import SearchStateAction, SearchStateGoal
from wr_logic_shortrange.msg import ShortRangeAction, ShortRangeGoal
from  wr_logic_ai.color_matrix import COLOR_NONE, COLOR_AUTONOMOUS, COLOR_COMPLETE, COLOR_ERROR, set_matrix_color
from wr_logic_ai.coordinate_manager import CoordinateManager

class St_Longrange_Complete(State):
    def __init__(self):
        State.__init__(self, outcomes=['longrange_only', 'shortrange'])

    def execute(self, userdata):
        # Checks if the waypoint requires target search and approach
        if CoordinateManager.get_coordinate()["target"] != "none":
            return 'longrange_only'
        else:
            return 'shortrange'

class St_Shortrange_Setup(State):
    def __init__(self):
        State.__init__(self, outcomes=['shortrange'], output_keys=['shortrange_target_type'])

    def execute(self, userdata):
        if CoordinateManager.get_coordinate()["target"] == "none":
            userdata.shortrange_target_type = ShortRangeGoal.TARGET_TYPE_GPS_ONLY
        else:
            userdata.shortrange_target_type = ShortRangeGoal.TARGET_TYPE_VISION
        return 'shortrange'

class St_Waypoint_Complete(State):
    def __init__(self):
        State.__init__(self, outcomes=['next_waypoint', 'complete'], output_keys=['target_lat', 'target_long'])

    # Defined for lambda function, not part of state machine architecture
    def _blink_complete(self) -> None:
        # Blinks the color off for 0.5 sec, the other 0.5 sec we sleep
        set_matrix_color(COLOR_COMPLETE)

    def execute(self, userdata):
        # Runs every one second, this blinks the LED to say we finished short range
        self._complete_blinker = rospy.Timer(
            rospy.Duration.from_sec(1), lambda _: self._blink_complete()
        )

        # The code will stay here until the user inputs a value ("c").
        while True:
            rospy.loginfo("Enter c to continue")
            if input("Enter c to continue: ") == "c":
                break

        self._complete_blinker.shutdown()

        # If there is a next coordinate, go back to long range. If not, it is complete
        if CoordinateManager.next_coordinate():
            return 'complete'
        else:
            userdata.target_lat = CoordinateManager.get_coordinate()["lat"]
            userdata.target_long = CoordinateManager.get_coordinate()["long"]
            return 'next_waypoint'

def main():
    sm = StateMachine(['succeeded','aborted','preempted'])
    CoordinateManager.read_coordinates_file()
    rospy.init_node("nav_state_machine", anonymous=False)
    rospy.loginfo("running state machine")
    sm.userdata.target_lat = CoordinateManager.get_coordinate()["lat"]
    sm.userdata.target_long = CoordinateManager.get_coordinate()["long"]

    with sm:
        # st_init_compass
        StateMachine.add('st_init_compass', 
                         SimpleActionState('InitCompassActionServer',
                                           InitCompassAction,
                                           goal = InitCompassGoal()
                        ),
                         transitions={'succeded':'st_longrange',
                                      'aborted':'st_init_compass'})
                       
        # st_longrange
        StateMachine.add('st_longrange', 
                         SimpleActionState('LongRangeActionServer',
                                           LongRangeAction,
                                           goal_slots=['target_lat', 'target_long']),
                         transitions={'succeeded':'st_longrange_complete',
                                      'aborted':'st_init_compass',
                                      'preempted':'st_init_compass'},
                         remapping={'target_lat':'target_lat',
                                    'target_long':'target_long'})

        # st_longrange_complete
        StateMachine.add('st_longrange_complete', St_Longrange_Complete(),
                         transitions={'longrange_only':'st_waypoint_complete',
                                      'shortrange':'st_search'})

        # st_search
        StateMachine.add('st_search', 
                         SimpleActionState('SearchActionServer', 
                                           SearchStateAction, 
                                           goal = SearchStateGoal()),
                         transitions={'succeeded':'st_shortrange_setup',
                                      'aborted':'st_init_compass',
                                      'preempted':'st_init_compass'})

        # st_shortrange_setup
        StateMachine.add('st_shortrange_setup', St_Shortrange_Setup(),
                         transitions={'shortrange':'st_shortrange'},
                         remapping={'shortrange_target_type':'shortrange_target_type'})

        # st_shortrange
        StateMachine.add('st_shortrange', 
                         SimpleActionState('ShortRangeActionServer', 
                                           ShortRangeAction, 
                                           goal_slots=['target_type']),
                         transitions={'succeeded':'st_waypoint_complete',
                                      'aborted':'st_search',
                                      'preempted':'st_search'},
                         remapping={'target_type':'shortrange_target_type'})

        # st_waypoint_complete
        StateMachine.add('st_waypoint_complete', St_Waypoint_Complete(),
                         transitions={'next_waypoint':'st_init_compass',
                                      'complete':'succeeded'},
                         remapping={'target_lat':'target_lat',
                                    'target_long':'target_long'})
        
    sm.execute()

if __name__ == "__main__":
    main()