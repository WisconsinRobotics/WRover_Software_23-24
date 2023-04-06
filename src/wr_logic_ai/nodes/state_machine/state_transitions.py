from state_machine import StateMachine
from navigation_events import *
from navigation_states import *

import os
import rospy
import json
from pathlib import Path
from std_msgs.msg import String

rospy.init_node('state_machine_driver', anonymous = False)

eventDic = {
    'evSuccess': evSuccess,
    'evNotComplete': evNotComplete,
    'evError': evError,
    'evWaiting': evWaiting,
    'evNotWaiting': evNotWaiting,
    'evKeepGoing' : evKeepGoing
}
stateMachine = StateMachine()
coordinate_manager = Coordinate_manager()


def init() -> None:
    rospy.Subscriber('/state_machine', String, process_event)
    #addTransition(current_state: State, transition_event: Event, future_state: State)
    stateMachine.add_transition(stInit, evKeepGoing, stLongRange)#TODO Change success to something better (keepGoing)
    stInit.set_coordinate_manager(coordinate_manager)
    stateMachine.add_transition(stLongRange, evNotComplete, stLongRange)
    stateMachine.add_transition(stLongRange, evSuccess, stShortRange)
    stateMachine.add_transition(stLongRange, evError, stError)
    stLongRange.set_coordinate_manager(coordinate_manager)
    stateMachine.add_transition(stError, evKeepGoing, stLR_Recovery) 
    stError.set_coordinate_manager(coordinate_manager)
    stateMachine.add_transition(stLR_Recovery, evError, stError)
    stateMachine.add_transition(stLR_Recovery, evNotComplete, stLR_Recovery)
    stateMachine.add_transition(stLR_Recovery, evSuccess, stLongRange)
    stLR_Recovery.set_coordinate_manager(coordinate_manager)
    stateMachine.add_transition(stShortRange, evNotComplete, stShortRange)
    stateMachine.add_transition(stShortRange, evError, stLongRange)
    stateMachine.add_transition(stShortRange, evSuccess, stWaypointSuccess)
    stateMachine.add_transition(stWaypointSuccess, evWaiting, stWaypointSuccess)
    stateMachine.add_transition(stWaypointSuccess, evNotWaiting, stLongRange)
    stWaypointSuccess.set_coordinate_manager(coordinate_manager)
    

    stateMachine.start(stInit())
    
    
def get_target_coordinates():
    dirname = Path(__file__).parents[1]
    file_name = Path.joinpath(dirname, 'coordinates.json')
    coordinates = open(file_name, 'r').read()
    c_json = json.loads(coordinates)
    return c_json

def process_event(event):
    stateMachine.process_event(eventDic[event])

if __name__ == "__main__":
    init()
    
    
    