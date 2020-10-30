from wr_drive_msgs.msg import DriveTrainCmd
import math

def get_x(heading: float, target_angle: float):
    return heading - target_angle

def piecewise_linear(heading: float, target_angle: float):
    get_x(heading, target_angle)
    if x < 0:
        return DriveTrainCmd(left_value = max(2/90*x+1,-1),right_value = 1)
    else:
        return DriveTrainCmd(left_value = 1, right_value = max(-2/90*x+1,-1))

def logistic(heading: float, target_angle: float):
    get_x(heading, target_angle)
    return DriveTrainCmd(left_value = 2/(1+math.exp(-(x+45)/10))-1, right_value = 2/(1+math.exp((x-45)/10))-1)

def rounded_logistic(heading: float, target_angle: float):
    raw = logistic(heading, target_angle)
    prec = 2
    prec_val = 10**prec
    return DriveTrainCmd(left_value = round(raw.left_value*prec_val)/prec_val, right_value = round(raw.right_value*prec_val)/prec_val)

def piecewise_sinusoidal(heading: float, target_angle: float):
    get_x(heading, target_angle)
    if x < 0:
        return DriveTrainCmd(left_value = math.sin(math.pi/90*(x+45)) if x >= -90 else -1, right_value = 1)
    else:
        return DriveTrainCmd(left_value = 1, right_value = -math.sin(math.pi/90*(x-45)) if x <= 90 else 1)
