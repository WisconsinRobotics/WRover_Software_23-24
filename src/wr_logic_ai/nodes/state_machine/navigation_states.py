class State:
    def __init__(self) -> None:
        pass

    def enter(self) -> None:
        raise NotImplementedError("This is an abstract method")

    def exit(self) -> None:
        raise NotImplementedError("This is an abstract method")

class stInit(State):
    def __init__(self) -> None:
        super().__init__()

    def enter(self) -> None:
        pass

    def exit(self) -> None:
        pass

class stLongRange(State):
    def __init__(self) -> None:
        super().__init__()

    def enter(self) -> None:
        pass

    def exit(self) -> None:
        pass

class stError(State):
    def __init__(self) -> None:
        super().__init__()

    def enter(self) -> None:
        pass

    def exit(self) -> None:
        pass

class stLR_Recovery(State):
    def __init__(self) -> None:
        super().__init__()

    def enter(self) -> None:
        pass

    def exit(self) -> None:
        pass

class stShortRange(State):
    def __init__(self) -> None:
        super().__init__()

    def enter(self) -> None:
        pass

    def exit(self) -> None:
        pass

class stWaypointSuccess(State):
    def __init__(self) -> None:
        super().__init__()

    def enter(self) -> None:
        pass

    def exit(self) -> None:
        pass