import time

class Watchdog:
    def __init__(self, timeout: float):
        self._timeout = timeout
        self.pet()

    def pet(self):
        self._lastTime = time.clock_gettime(0)

    def isMad(self):
        return time.clock_gettime(0) - self._lastTime >= self._timeout

