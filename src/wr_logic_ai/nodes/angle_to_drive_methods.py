from wr_drive_msgs.msg import DriveTrainCmd
import math

# Used to get the difference between the target angle and the current heading.  Used to abstract left from right.
def get_x(heading: float, target_angle: float) -> float:
    x = heading - target_angle
    # Placeholder for extra logic on computing x during development
    return x

# Creates a DriveTrainCmd using piecewise linear functions that are linear on 
# the domain [-90, 0] or [0, 90] (depending on the side) and extend 1 and -1 outside of that domain
def piecewise_linear(heading: float, target_angle: float) -> DriveTrainCmd:
    x = get_x(heading, target_angle)

    #Takes fastest angle to turn robot
    if x < 0:
        return DriveTrainCmd(left_value = max(2/20*x+1,-1),right_value = 1)
    else:
        return DriveTrainCmd(left_value = 1, right_value = max(-2/20*x+1,-1))

    

    '''
    if x < 0:
        return DriveTrainCmd(left_value = max(2/90*x+1,-1),right_value = 1)
    else:
        return DriveTrainCmd(left_value = 1, right_value = max(-2/90*x+1,-1))
    '''

# Creates a DriveTrainCmd using sigmoid functions that are approximately linear on 
# the domain [-90, 0] or [0,90] (depending on the side) on the range (-1,1)
def logistic(heading: float, target_angle: float) -> DriveTrainCmd:
    x = get_x(heading, target_angle)
    return DriveTrainCmd(left_value = 2/(1+math.exp(-(x+45)/10))-1, right_value = 2/(1+math.exp((x-45)/10))-1)

# Uses the logistic(float, float) function to create a DriveTrainCmd rounded to a 
# specified precision to produce the sigmoid shape without the messy numbers
def rounded_logistic(heading: float, target_angle: float, prec: int = 2) -> DriveTrainCmd:
    raw = logistic(heading, target_angle)
    prec_val = 10**prec
    return DriveTrainCmd(left_value = round(raw.left_value*prec_val)/prec_val, right_value = round(raw.right_value*prec_val)/prec_val)

# Creates a DriveTrainCmd using piecewise sinusoids (sine function) on the 
# domain [-90, 0] or [0, 90] (depending on the side) and extend 1 and -1 outside of that domain
def piecewise_sinusoidal(heading: float, target_angle: float) -> DriveTrainCmd:
    x = get_x(heading, target_angle)
    if x < 0:
        return DriveTrainCmd(left_value = math.sin(math.pi/90*(x+45)) if x >= -90 else -1, right_value = 1)
    else:
        return DriveTrainCmd(left_value = 1, right_value = -math.sin(math.pi/90*(x-45)) if x <= 90 else 1)
