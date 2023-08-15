# wr_control_drive_arm

@defgroup wr_control_drive_arm wr_control_drive_arm
@brief Translates setpoint motion commands to hardware commands for the arm drive

## Purpose

The logic layer for the arm does Inverse Kinematics (IK) to provide a more intuitive end-effector controller, compared to a by-joint controller, where the operator controls the velocity of each joint independently.  The logic required to perform IK is relatively complex and expensive, it has been offloaded to MoveIt, a ROS package specializing in IK calculation and execution.

However, MoveIt only does the calculation.

![sequence diagram](seq.png)
