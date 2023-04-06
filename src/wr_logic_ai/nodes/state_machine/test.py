from state_machine import StateMachine
from navigation_events import Event
from navigation_states import State
from coordinate_manager import Coordinate_manager
import rospy

class test_a(State):
    def __init__(self) -> None:
        super().__init__()

    def enter(self) -> None:
        print("entered state a")

    def exit(self) -> None:
        print("exited state a")

class test_b(State):
    def __init__(self) -> None:
        super().__init__()
    
    def enter(self) -> None:
        print("entered state b")

    def exit(self) -> None:
        print("exited state b")

class a_to_b(Event):
    def __init__(self) -> None:
        super().__init__()

class b_to_a(Event):
    def __init__(self) -> None:
        super().__init__()

if __name__ == "__main__":
    # stateMachine = StateMachine()
    # stateMachine.add_transition(test_a, a_to_b, test_b)
    # stateMachine.add_transition(test_b, b_to_a, test_a)
    # stateMachine.start(test_a())
    
    # while True:
    #     stateMachine.process_event(a_to_b)
    #     stateMachine.process_event(b_to_a)

    Coordinate_manager()
    print(Coordinate_manager.get_coordinate())
    Coordinate_manager.next_line()
    print(Coordinate_manager.get_coordinate())