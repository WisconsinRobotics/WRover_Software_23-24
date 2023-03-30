class Event:
    def __init__(self) -> None:
        pass

class evSuccess(Event):
    def __init__(self) -> None:
        pass

class evNotComplete(Event):
    def __init__(self) -> None:
        pass

class evError(Event):
    def __init__(self) -> None:
        pass

class evWaiting(Event):
    def __init__(self) -> None:
        pass

class evNotWaiting(Event):
    def __init__(self) -> None:
        pass

class evKeepGoing(Event):
    def __init__(self) -> None:
        pass