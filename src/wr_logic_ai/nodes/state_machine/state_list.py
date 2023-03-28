# StateMachine/mouse/StateAction.py

class StateAction:
    def __init__(self, action):
        self.action = action
    def __str__(self): 
        return self.action
    def __cmp__(self, other):
        return (self < other) - (other < self)
    # Necessary when __cmp__ or __eq__ is defined
    # in order to make this class usable as a
    # dictionary key:
    def __hash__(self):
        return hash(self.action)

# Static fields; an enumeration of instances:
StateAction.init = StateAction("robot init")
StateAction.not_completed_LR = StateAction("long_range not completed")
StateAction.succes_LR = StateAction("long_range success")
StateAction.error_LR = StateAction("long_range error")
StateAction.not_completed_SR = StateAction("short_range not completed")
StateAction.error_LR = StateAction("short_range error")
StateAction.success_LR = StateAction("short_range success")
StateAction.waypoint_success_waiting = StateAction("waypoint_success waiting")
StateAction.waypoint_success_not_waiting = StateAction("waypoint_success not waiting")
StateAction.error = StateAction("error") #Go to LR Recovery
StateAction.LR_recovery_not_complete = StateAction("LR_recovery not complete")
StateAction.LR_recovery_error = StateAction("LR_recovery error")
StateAction.LR_recovery_success = StateAction("LR_recovery success")