from enum import Enum, Flag
from typing import Tuple

from wr_logic_ai.msg import VisionTarget


class ShortrangeStateEnum(Flag):
    FAIL = 1
    SUCCESS = 2
    VISION_DRIVE_POST = 4
    VISION_DRIVE_GATE = 8
    ENCODER_DRIVE = 16
    TERMINATING = FAIL | SUCCESS


class ShortrangeState:
    def run(self) -> Tuple[ShortrangeStateEnum, int]:
        raise NotImplementedError


class TargetCache:
    """
    Class that contains a VisionTarget and timestamp
    """

    def __init__(self, timestamp: float, msg: VisionTarget):
        self.timestamp = timestamp
        self.msg = msg
