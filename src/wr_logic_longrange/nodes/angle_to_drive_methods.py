"""@file
@defgroup wr_logic_ai_longrange_ai
@{
@defgroup wr_logic_ai_longrange_ai_angle_to_drive_methods Angle to Drive Calculations
@brief Helper class for converting angle headings to motor drive power
@details Intermediate logic layer used to convert rover target heading to signals to motor drive power.
@{
"""

from wr_drive_msgs.msg import DriveTrainCmd
import math


def get_x(heading: float, target_angle: float) -> float:
    """
    Used to get the difference between the target angle and the current heading.  Used to abstract
    left from right

    Args:
        heading (float): Current heading of the rover
        target_angle (float): Target heading of the destination

    Returns:
        float: The angular difference between the target angle and the current heading
    """
    x = heading - target_angle
    # Placeholder for extra logic on computing x during development
    return x


def piecewise_linear(heading: float, target_angle: float) -> DriveTrainCmd:
    """
    Creates a DriveTrainCmd using piecewise linear functions that are linear on the domain [-90, 0]
    or [0, 90] (depending on the side) and extend 1 and -1 outside of that domain

    @param heading (float): Current heading of the rover
    @param target_angle (float): Target heading of the destination
    @return DriveTrainCmd: Message object that contains the calculated drive powers
    """
    # TODO: Flip iputs and flip x

    x = get_x(heading, target_angle)

    # Takes fastest angle to turn robot
    if x < 0:
        return DriveTrainCmd(left_value=max(2 / 45 * x + 1, -1), right_value=1)
    else:
        return DriveTrainCmd(left_value=1, right_value=max(-2 / 45 * x + 1, -1))

    """
    if x < 0:
        return DriveTrainCmd(left_value = max(2/90*x+1,-1),right_value = 1)
    else:
        return DriveTrainCmd(left_value = 1, right_value = max(-2/90*x+1,-1))
    """


def logistic(heading: float, target_angle: float) -> DriveTrainCmd:
    """
    Creates a DriveTrainCmd using sigmoid functions that are approximately linear on the domain
    [-90, 0] or [0, 90] (depending on the side) on the range (-1,1)

    @param heading (float): Current heading of the rover
    @param target_angle (float): Target heading of the destination
    @return DriveTrainCmd: Message object that contains the calculated drive powers
    """

    x = get_x(heading, target_angle)
    return DriveTrainCmd(
        left_value=2 / (1 + math.exp(-(x + 45) / 10)) - 1,
        right_value=2 / (1 + math.exp((x - 45) / 10)) - 1,
    )


def rounded_logistic(
    heading: float, target_angle: float, prec: int = 2
) -> DriveTrainCmd:
    """
    Uses the logistic(float, float) function to create a DriveTrainCmd rounded to a specified
    precision to produce the sigmoid shape without the messy numbers

    @param heading (float): Current heading of the rover
    @param target_angle (float): Target heading of the destination
    @pram prec (int, optional): Precision value. Defaults to 2.
    @return DriveTrainCmd: Message object that contains the calculated drive powers
    """

    raw = logistic(heading, target_angle)
    prec_val = 10**prec
    return DriveTrainCmd(
        left_value=round(raw.left_value * prec_val) / prec_val,
        right_value=round(raw.right_value * prec_val) / prec_val,
    )


# Creates a DriveTrainCmd using piecewise sinusoids (sine function) on the
# domain [-90, 0] or [0, 90] (depending on the side) and extend 1 and -1 outside of that domain
def piecewise_sinusoidal(heading: float, target_angle: float) -> DriveTrainCmd:
    """
    Creates a DriveTrainCmd using piecewise sinusoids (sine function) on the domain [-90, 0] or
    [0, 90] (depending on the side) and extend 1 and -1 outside of that domain

    @param heading (float): Current heading of the rover
    @param target_angle (float): Target heading of the destination
    @return DriveTrainCmd: Message object that contains the calculated drive powers
    """

    x = get_x(heading, target_angle)
    if x < 0:
        return DriveTrainCmd(
            left_value=math.sin(math.pi / 90 * (x + 45)) if x >= -90 else -1,
            right_value=1,
        )
    else:
        return DriveTrainCmd(
            left_value=1,
            right_value=-math.sin(math.pi / 90 * (x - 45)) if x <= 90 else 1,
        )


## @}
## @}
