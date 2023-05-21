#!/usr/bin/env python

from __future__ import annotations
from statemachine import StateMachine, State
from wr_logic_ai.coordinate_manager import CoordinateManager
from wr_logic_ai.msg import NavigationState, LongRangeAction, LongRangeGoal, LongRangeActionResult
from wr_led_matrix.srv import led_matrix as LEDMatrix, led_matrixRequest as LEDMatrixRequest
from std_srvs.srv import Empty
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from wr_drive_msgs.msg import DriveTrainCmd

COLOR_AUTONOMOUS = LEDMatrixRequest(RED = 0, GREEN = 0, BLUE = 255)
COLOR_COMPLETE = LEDMatrixRequest(RED = 0, GREEN = 255, BLUE = 0)
COLOR_ERROR = LEDMatrixRequest(RED = 255, GREEN = 0, BLUE = 0)
COLOR_NONE = LEDMatrixRequest(RED = 0, GREEN = 0, BLUE = 0)

def set_matrix_color(color: LEDMatrixRequest) -> None:
    matrix_srv = rospy.ServiceProxy("/led_matrix", LEDMatrix)
    matrix_srv.wait_for_service()
    matrix_srv.call(COLOR_NONE)
    rospy.sleep(0.5)
    matrix_srv.call(color)

class NavStateMachine(StateMachine):
    # Defining states
    stInit = State(initial=True)
    stLongRange = State()
    stLongRangeRecovery = State()
    stShortRange = State()
    stWaypointSuccess = State()
    stComplete = State()

    # Defining events and transitions
    evSuccess = (stLongRange.to(stShortRange) | stLongRangeRecovery.to(
        stLongRange) | stShortRange.to(stWaypointSuccess))
    evError = (stLongRange.to(stLongRangeRecovery) | stLongRangeRecovery.to(
        stLongRangeRecovery) | stShortRange.to(stLongRange))
    evNotWaiting = stWaypointSuccess.to(stLongRange)
    evUnconditional = stInit.to(stLongRange)
    evComplete = stWaypointSuccess.to(stComplete)

    def __init__(self, mgr: CoordinateManager) -> None:
        self._mgr = mgr
        self.currentEvent = -1
        self.mux_pub = rospy.Publisher("/navigation_state", NavigationState, queue_size=1)
        self.mux_long_range = NavigationState()
        self.mux_long_range.state = NavigationState.NAVIGATION_STATE_LONG_RANGE
        self.mux_short_range = NavigationState()
        self.mux_short_range.state = NavigationState.NAVIGATION_STATE_SHORT_RANGE
        super(NavStateMachine, self).__init__()

    def init_calibrate(self, pub: rospy.Publisher, stop_time: float) -> None:
        set_matrix_color(COLOR_AUTONOMOUS)
        if rospy.get_time() < stop_time:
            pub.publish(DriveTrainCmd(left_value=0.3, right_value=-0.3))
        else:
            pub.publish(DriveTrainCmd(left_value=0, right_value=0))
            self.evUnconditional()

    def on_enter_stInit(self) -> None:
        print("\non enter stInit")
        rospy.loginfo("\non enter stInit")
        self._mgr.read_coordinates_file()

        pub = rospy.Publisher("/control/drive_system/cmd", DriveTrainCmd, queue_size=1)
        self._init_tmr = rospy.Timer(rospy.Duration.from_sec(0.1), lambda _: self.init_calibrate(pub, rospy.get_time() + 7))

    def on_exit_stInit(self) -> None:
        self._init_tmr.shutdown()
        if (self._mgr.next_coordinate()):
            self.evComplete()

    def longRangeActionComplete(self, state: GoalStatus, _: LongRangeActionResult) -> None:
        if state == GoalStatus.SUCCEEDED:
            self.evSuccess()
        elif state == GoalStatus.ABORTED:
            self.evError()
            

    def on_enter_stLongRange(self) -> None:
        print("\non enter stLongRange")
        rospy.loginfo("\non enter stLongRange")
        self.timer = rospy.Timer(rospy.Duration(0.2), lambda _: self.mux_pub.publish(self.mux_long_range)) 
        
        # enter autonomous mode
        set_matrix_color(COLOR_AUTONOMOUS)

        client = actionlib.SimpleActionClient("LongRangeActionServer", LongRangeAction)
        client.wait_for_server()
        goal = LongRangeGoal(target_lat = self._mgr.get_coordinate()["lat"], target_long = self._mgr.get_coordinate()["long"])
        client.send_goal(goal, done_cb= \
                         lambda status, result: \
                            self.longRangeActionComplete(status, result))
        

    def on_exit_stLongRange(self) -> None:
        print("Exting Long Range")
        rospy.loginfo("Exting Long Range")
        self.timer.shutdown()

    def longRangeRecoveryActionComplete(self, state: GoalStatus, _: LongRangeActionResult) -> None:
        if state == GoalStatus.SUCCEEDED:
            self.evSuccess()
        elif state == GoalStatus.ABORTED:
            self.evError()

    def on_enter_stLongRangeRecovery(self) -> None:
        print("\non enter stLongRangeRecovery")
        rospy.loginfo("\non enter stLongRangeRecovery")

        set_matrix_color(COLOR_ERROR)

        if self._mgr is None:
            raise ValueError
        else:
            self._mgr.previous_coordinate()
            print(self._mgr.get_coordinate())  
            self.timer = rospy.Timer(rospy.Duration(0.2), lambda _: self.mux_pub.publish(self.mux_long_range))           
            client = actionlib.SimpleActionClient("LongRangeActionServer", LongRangeAction)
            client.wait_for_server()
            goal = LongRangeGoal(target_lat = self._mgr.get_coordinate()["lat"], target_long = self._mgr.get_coordinate()["long"])
            client.send_goal(goal, done_cb= \
                             lambda status, result: \
                                self.longRangeRecoveryActionComplete(status, result))

    def on_exit_stLongRangeRecovery(self) -> None:
        self.timer.shutdown()

    def on_enter_stShortRange(self) -> None:
        print("\non enter stShortRange")
        rospy.loginfo("\non enter stShortRange")

        set_matrix_color(COLOR_AUTONOMOUS)

        if CoordinateManager.short_range_complete() != True:
            print("Short Range Not Complete")
            rospy.loginfo("Short Range Not Complete")
            self.timer = rospy.Timer(rospy.Duration(0.2), lambda _: self.mux_pub.publish(self.mux_short_range))
        else:
            print("Short Range Complete")
            rospy.loginfo("Short Range Complete")
            self.evSuccess()
        self.evSuccess()

    def on_exit_stShortRange(self) -> None:
        # self.timer.shutdown()
        pass

    def on_enter_stWaypointSuccess(self) -> None:
        print("\non enter stWaypointSuccess")

        set_matrix_color(COLOR_COMPLETE)

        if self._mgr is None:
            raise ValueError
        else:
            rospy.wait_for_service('wait_for_user_input_service')
            try:
                wait_for_user_input = rospy.ServiceProxy('wait_for_user_input_service', Empty)
                wait_for_user_input()
            except rospy.ServiceException as e:
                print(e)
            if (self._mgr.next_coordinate()):
                print("Should Enter event complete")
                self.evComplete()
            else:
                self.evNotWaiting()

    def on_exit_stWaypointSuccess(self) -> None:
        pass  # Frash lights that we are goint to next waypoint

    def on_enter_stComplete(self) -> None:
        print("We finished, wooooo")

if __name__ == "__main__":
    rospy.init_node('nav_state_machine', anonymous=False)
    statemachine = NavStateMachine(CoordinateManager())
    rospy.spin()
