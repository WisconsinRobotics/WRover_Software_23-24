from state_machine import StateMachine
from navigation_events import *
from navigation_states import *

if __name__ == "__main__":

    #addTransition(current_state: State, transition_event: Event, future_state: State)
    stateMachine = StateMachine()
    stateMachine.add_transition(stInit, evKeepGoing, stLongRange)#TODO Change success to something better (keepGoing)
    stateMachine.add_transition(stLongRange, evNotComplete, stLongRange)
    stateMachine.add_transition(stLongRange, evSuccess, stShortRange)
    stateMachine.add_transition(stError, evKeepGoing, stLR_Recovery) 
    stateMachine.add_transition(stLR_Recovery, evError, stError)
    stateMachine.add_transition(stLR_Recovery, evNotComplete, stLR_Recovery)
    stateMachine.add_transition(stLR_Recovery, evSuccess, stLongRange)
    stateMachine.add_transition(stShortRange, evNotComplete, stShortRange)
    stateMachine.add_transition(stShortRange, evError, stLongRange)
    stateMachine.add_transition(stShortRange, evSuccess, stWaypointSuccess)
    stateMachine.add_transition(stWaypointSuccess, evWaiting, stWaypointSuccess)
    stateMachine.add_transition(stWaypointSuccess, evNotWaiting, stLongRange)
    stateMachine.start(stInit)