from navigation_events import *
from navigation_states import *

class StateMachine:
    def __init__(self) -> None:
        self.transitions = {}
        self.current_state = None

    def start(self, init_state: State) -> None:
        self.current_state = init_state
        self.current_state.enter()

    def process_event(self, event: Event) -> None:
        if event in self.transitions[type(self.current_state)]:
            self.current_state.exit()
            self.current_state = self.transitions[type(self.current_state)][event]()
            self.current_state.enter()

    def add_transition(self, current_state: State, transition_event: Event, future_state: State) -> None:
        if(current_state in self.transitions):
            #enter new inner transition into the state
            self.transitions[current_state][transition_event] = future_state
        else:
            #Make new dictionary to add to values of outer state
            innerTransition = {}
            innerTransition[transition_event] = future_state

            #Add first transition to state
            self.transitions[current_state] = innerTransition