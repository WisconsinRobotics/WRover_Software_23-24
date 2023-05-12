#!/usr/bin/env python

from statemachine import StateMachine, State
from wr_logic_ai.coordinate_manager import CoordinateManager
from wr_logic_ai.msg import NavigationState, LongRangeAction, LongRangeGoal
from std_srvs.srv import Empty
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from wr_drive_msgs.msg import DriveTrainCmd

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

    def on_enter_stInit(self) -> None:
        print("\non enter stInit")
        self._mgr.read_coordinates_file()

        pub = rospy.Publisher("/control/drive_system/cmd", DriveTrainCmd, queue_size=1)
        end = rospy.get_time() + 7
        while rospy.get_time() < end:
            pub.publish(DriveTrainCmd(left_value=0.4, right_value=-0.4))
            rospy.sleep(0.1)
        pub.publish(DriveTrainCmd(left_value=0, right_value=0))

        self.evUnconditional()

    def on_exit_stInit(self) -> None:
        if (self._mgr.next_coordinate()):
            self.evComplete

    def on_enter_stLongRange(self) -> None:
        print("\non enter stLongRange")
        self.timer = rospy.Timer(rospy.Duration(0.2), lambda _: self.mux_pub.publish(self.mux_long_range)) 
        client = actionlib.SimpleActionClient("LongRangeActionServer", LongRangeAction)
        client.wait_for_server()
        goal = LongRangeGoal(target_lat = self._mgr.get_coordinate()["lat"], target_long = self._mgr.get_coordinate()["long"])
        as_state = client.send_goal_and_wait(goal)
        if as_state == GoalStatus.SUCCEEDED:
            self.evSuccess()
        elif as_state == GoalStatus.ABORTED:
            self.evError()

    def on_exit_stLongRange(self) -> None:
        print("Exting Long Range")
        self.timer.shutdown()

    def on_enter_stLongRangeRecovery(self) -> None:
        print("\non enter stLongRangeRecovery")
        if self._mgr is None:
            raise ValueError
        else:
            self._mgr.previous_coordinate()
            print(self._mgr.get_coordinate())  
            self.timer = rospy.Timer(rospy.Duration(0.2), lambda _: self.mux_pub.publish(self.mux_long_range))           
            client = actionlib.SimpleActionClient("LongRangeActionServer", LongRangeAction)
            client.wait_for_server()
            goal = LongRangeGoal(target_lat = self._mgr.get_coordinate()["lat"], target_long = self._mgr.get_coordinate()["long"])
            as_state = client.send_goal_and_wait(goal)
            if as_state == GoalStatus.SUCCEEDED:
                self.evSuccess()
                self._mgr.next_coordinate()
            elif as_state == GoalStatus.ABORTED:
                self.evError()

    def on_exit_stLongRangeRecovery(self) -> None:
        self.timer.shutdown()

    def on_enter_stShortRange(self) -> None:
        # print("\non enter stShortRange")
        # if CoordinateManager.short_range_complete() != True:
        #     print("Short Range Not Complete")
        #     self.timer = rospy.Timer(rospy.Duration(0.2), lambda _: self.mux_pub.publish(self.mux_short_range))
        # else:
        #     print("Short Range Complete")
        #     self.evSuccess()
        self.evSuccess()

    def on_exit_stShortRange(self) -> None:
        # self.timer.shutdown()
        pass

    def on_enter_stWaypointSuccess(self) -> None:
        print("\non enter stWaypointSuccess")
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
