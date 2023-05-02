from enum import Enum

from wr_logic_ai.msg import TargetMsg


class ShortrangeAIGoal(Enum):
    NO_TARGET = 0,
    ONE_TARGET = 1,
    TWO_TARGETS = 2


class ShortrangeAIStates(Enum):
    FAIL = 0,
    SUCCESS = 1,
    VISION_DRIVE_POST = 2,
    VISION_DRIVE_GATE = 3,
    ENCODER_DRIVE = 4


class ShortrangeState:
    def run(self) -> ShortrangeAIStates:
        raise NotImplementedError


class TargetCache:
    """
    Class that contains a TargetMsg and timestamp
    """
    def __init__(self, timestamp: float, msg: TargetMsg):
        self.timestamp = timestamp
        self.msg = msg
