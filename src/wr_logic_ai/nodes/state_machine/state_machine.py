from statemachine import StateMachine, State
from coordinate_manager import CoordinateManager
from wr_logic_ai.msg import TargetMsg
from wr_logic_ai.msg import NavigationStateMsg
import rospy

class NavStateMachine(StateMachine):
    rospy.init_node('nav_state_machine', anonymous=False)
    stInit = State(initial=True)
    stLongRange = State()
    stLongRangeRecovery = State()
    stShortRange = State()
    stWaypointSuccess = State()

    evSuccess = (stLongRange.to(stShortRange) | stLongRangeRecovery.to(stLongRange) | stShortRange.to(stWaypointSuccess))
    evError = (stLongRange.to(stLongRangeRecovery) | stLongRangeRecovery.to(stLongRangeRecovery) | stShortRange.to(stLongRange))
    evNotWaiting = stWaypointSuccess.to(stLongRange)
    evUnconditional = stInit.to(stLongRange)

    def __init__(self, mgr: CoordinateManager) -> None:
        self._mgr = mgr
        rospy.Subscriber('/nav_state_msg', NavigationStateMsg, lambda msg: self.process_event(msg))
        super(NavStateMachine, self).__init__()


    def process_event(self, data: NavigationStateMsg):
        print(data.nav_state_type)
        if data.nav_state_type == NavigationStateMsg.NAV_STATE_TYPE_SUCCESS:
            self.evSuccess()
        elif data.nav_state_type == NavigationStateMsg.NAV_STATE_TYPE_ERROR:
            self.evError()
        else:
            raise TypeError
    
    def on_enter_stInit(self) -> None:
        print("\non enter stInit")
        self._mgr.read_coordinates_file()
        self.evUnconditional()

    def on_exit_stInit(self) -> None:
        self._mgr.next_line()

    def on_enter_stLongRange(self)->None:
        print("\non enter stLongRange")
        # self.timer = rospy.Timer(rospy.Duration(0.2), self.publish)
        #self.pub_nav = rospy.Publisher('/target_coord', TargetMsg, queue_size=1)
        print(self._mgr.get_coordinate())
        #rospy.spin()

    def on_exit_stLongRange(self) -> None:
        #self.timer.shutdown()
        pass

    def on_enter_stLongRangeRecovery(self) -> None:
        print("\non enter stLongRangeRecovery")
        if self._mgr is None:
            raise ValueError
        else:
            self._mgr.previous_line()
            print(self._mgr.get_coordinate())
            #self.timer = rospy.Timer(rospy.Duration(0.2), self.publish)
            #self.pub_nav = rospy.Publisher('/target_coord', TargetMsg, queue_size=1)
            #rospy.spin()

    def on_exit_stLongRangeRecovery(self) -> None:
        #self.timer.shutdown()
        self._mgr.next_line()
            
    def on_enter_stShortRange(self) -> None:
        print("\non enter stShortRange")
        if CoordinateManager.short_range_complete() != True:
            pass
        
    def on_exit_stShortRange(self) -> None:
        pass
        
    def on_enter_stWaypointSuccess(self) -> None:
        print("\non enter stWaypointSuccess")
        if self._mgr is None:
            raise ValueError
        else:
            self._mgr.next_line()
            self.evNotWaiting()

    def on_exit_stWaypointSuccess(self) -> None:
        pass #Frash lights that we are goint to next waypoint

    def publish(self, timer):    
        # Publish to obstacle avoidance with the target coordinates
        target_coords = TargetMsg()
        target_coords.target_lat = self._mgr.get_coordinate()['lat']
        target_coords.target_long = self._mgr.get_coordinate()['long']
        target_coords.target_type = self._mgr.get_coordinate()['target_type']
        self.pub_nav.publish(target_coords)
        # TODO: Publish to topic /navigation_state