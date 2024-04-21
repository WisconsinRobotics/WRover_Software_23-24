import rospy
from smach import StateMachine, State
from smach_ros import SimpleActionState
from std_srvs.srv import Empty

from wr_logic_longrange.msg import LongRangeAction, LongRangeGoal
from wr_logic_search.msg import SearchStateAction, SearchStateGoal
from wr_logic_shortrange.msg import ShortRangeAction, ShortRangeGoal
from  wr_logic_ai.color_matrix import COLOR_NONE, COLOR_AUTONOMOUS, COLOR_COMPLETE, COLOR_ERROR, set_matrix_color
from wr_logic_ai.coordinate_manager import CoordinateManager

class St_Longrange_Complete(State):
    def __init__(self):
        State.__init__(self, outcomes=['longrange_only', 'shortrange'], input_keys=['coord_mgr'])

    def execute(self, userdata):
        # Checks if the waypoint requires target search and approach
        if (userdata.coord_mgr.get_coordinate()["target"] != "none"):
            return 'longrange_only'
        else:
            return 'shortrange'

class St_Waypoint_Complete(State):
    def __init__(self):
        State.__init__(self, outcomes=['next_waypoint', 'complete'], input_keys=['coord_mgr'])

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
            if input("Enter c to continue: ") == "c":
                break

        self._complete_blinker.shutdown()

        # If there is a next coordinate, go back to long range. If not, it is complete
        if userdata.coord_mgr.next_coordinate():
            return 'complete'
        else:
            return 'next_waypoint'

def main():
    sm = StateMachine(['succeeded','aborted','preempted'])
    sm.userdata.coord_mgr = CoordinateManager()
    sm.userdata.coord_mgr.read_coordinates_file()
    rospy.init_node(rospy.init_node("nav_state_machine", anonymous=False))

    with sm:
        # st_longrange
        longrange_goal = LongRangeGoal()
        longrange_goal.target_lat = sm.userdata.coord_mgr.get_coordinate()["lat"]
        longrange_goal.target_long = sm.userdata.coord_mgr.get_coordinate()["long"]
        StateMachine.add('st_longrange', 
                         SimpleActionState('LongRangeActionServer',
                                           LongRangeAction,
                                           goal=longrange_goal),
            transitions={'succeeded':'st_longrange_complete',
                         'aborted':'st_longrange',
                         'preempted':'st_longrange'})

        # st_longrange_complete
        StateMachine.add('st_longrange_complete', St_Longrange_Complete(),
                         transitions={'longrange_only':'st_waypoint_complete',
                                      'shortrange':'st_search'},
                         remapping={'coord_mgr':'coord_mgr'})

        # st_search
        StateMachine.add('st_search', 
                         SimpleActionState('SearchActionServer', 
                                           SearchStateAction, 
                                           goal = SearchStateGoal()),
                         transitions={'succeeded':'st_shortrange',
                                      'aborted':'st_longrange',
                                      'preempted':'st_longrange'})

        # st_shortrange
        shortrange_goal = ShortRangeGoal()
        if sm.userdata.coord_mgr.get_coordinate()["target"] == "none":
            shortrange_goal.target_type = ShortRangeGoal.TARGET_TYPE_GPS_ONLY
        else:
            shortrange_goal.target_type = ShortRangeGoal.TARGET_TYPE_VISION
        StateMachine.add('st_shortrange', 
                         SimpleActionState('ShortRangeActionServer', 
                                           ShortRangeAction, 
                                           goal = shortrange_goal),
                         transitions={'succeeded':'st_waypoint_complete',
                                      'aborted':'st_search',
                                      'preempted':'st_search'})

        # st_waypoint_complete
        StateMachine.add('st_waypoint_complete', St_Waypoint_Complete(),
                         transitions={'next_waypoint':'st_longrange',
                                      'complete':'succeeded'},
                         remapping={'coord_mgr':'coord_mgr'})
        
    sm.execute()

if __name__ == "__main__":
    main()