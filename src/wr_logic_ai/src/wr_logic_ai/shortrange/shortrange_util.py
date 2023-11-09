##@defgroup wr_shortrange_ai
# @{
# @defgroup wr_shortrange_ai_util Utility Classes
# @brief Miscellaneous classes used to implement the shortrange state machine
# @{

from enum import Enum, Flag
from typing import Tuple

from wr_logic_ai.msg import VisionTarget


class ShortrangeStateEnum(Flag):
    """
    Enum containing shortrange navigation states
    """

    ## Shortrange navigation failed
    FAIL = 1
    ## Shortrange navigation succeeded
    SUCCESS = 2
    ## Shortrange navigation is driving using vision data
    VISION_DRIVE = 4
    ## Terminating states of shortrange navigation
    TERMINATING = FAIL | SUCCESS


class ShortrangeState:
    """
    An abstract class that should be implemented for shortrange navigation states
    """

    def run(self) -> ShortrangeStateEnum:
        """
        Virtual function that the shortrange state machine runs

        @return ShortrangeStateEnum: The next state to execute
        """
        raise NotImplementedError


class TargetCache:
    """
    A class that contains a VisionTarget and timestamp

    This class is used to track the last valid VisionTarget in VisionNavigation.
    """

    def __init__(self, timestamp: float, msg: VisionTarget):
        ## Timestamp of the VisionTarget
        self.timestamp = timestamp
        ## Cached VistionTarget message
        self.msg = msg


## @}
# @}
