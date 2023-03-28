# StateMachine/mousetrap2/MouseTrap2Test.py
# A better mousetrap using tables
import string, sys
sys.path += ['../stateMachine', '../mouse']
import rospy
from state_list import StateAction

# StateMachine/State.py
# A State has an operation, and can be moved
# into the next State given an Input:

def initialize():
    rospy.Subscriber('/is_LR_complete', bool, check_LR_status)
    rospy.Subscriber('/is_SR_complete', bool, check_SR_status)

class State:
    def run(self):
        assert 0, "run not implemented"
    def next(self, input):
        assert 0, "next not implemented"


# StateMachine/StateMachine.py
# Takes a list of Inputs to move from State to
# State using a template method.

class StateMachine:
    def __init__(self, initialState):
        self.currentState = initialState
        self.currentState.run()
    # Template method:
    def runAll(self, inputs):
        for i in inputs:
            print(i)
            self.currentState = self.currentState.next(i)
            self.currentState.run()


class StateT(State):
    def __init__(self):
        self.transitions = None
    def next(self, input):
        if self.transitions.has_key(input):
            return self.transitions[input]
        else:
            raise "Input not supported for current state"

class Init(StateT):
    def run(self):
        pass
    def next(self,input):
        if not self.transitions:
            Action.luring


class check_LR_status(StateT):
    def run(self):
        if completed == True:
            pass
        else:
            mux_pub.publish(2) #Short range is not finished
    def next(self, input):
        # Lazy initialization:
        if not self.transitions:
            self.transitions = {
              MouseAction.appears : MouseTrap.luring
            }
        return StateT.next(self, input)

class Luring(StateT):
    def run(self):
        print("Luring: Presenting Cheese, door open")
    def next(self, input):
        # Lazy initialization:
        if not self.transitions:
            self.transitions = {
              MouseAction.enters : MouseTrap.trapping,
              MouseAction.runsAway : MouseTrap.waiting
            }
        return StateT.next(self, input)

class Trapping(StateT):
    def run(self):
        print("Trapping: Closing door")
    def next(self, input):
        # Lazy initialization:
        if not self.transitions:
            self.transitions = {
              MouseAction.escapes : MouseTrap.waiting,
              MouseAction.trapped : MouseTrap.holding
            }
        return StateT.next(self, input)

class Holding(StateT):
    def run(self):
        print("Holding: Mouse caught")
    def next(self, input):
        # Lazy initialization:
        if not self.transitions:
            self.transitions = {
              MouseAction.removed : MouseTrap.waiting
            }
        return StateT.next(self, input)

class Action(StateMachine):
    def __init__(self):
        # Initial state
        StateMachine.__init__(self, MouseTrap.waiting)

# Static variable initialization:


moves = map(string.strip,
  open("../mouse/MouseMoves.txt").readlines())
mouseMoves = map(MouseAction, moves)
MouseTrap().runAll(mouseMoves)