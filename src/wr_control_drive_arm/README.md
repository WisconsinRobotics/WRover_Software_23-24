# wr_control_drive_arm

@defgroup wr_control_drive_arm wr_control_drive_arm
@brief Translates setpoint motion commands to hardware commands for the arm drive

## Purpose

The logic layer for the arm does Inverse Kinematics (IK) to provide a more intuitive end-effector controller, compared to a by-joint controller, where the operator controls the velocity of each joint independently.  The logic required to perform IK is relatively complex and expensive, it has been offloaded to MoveIt, a ROS package specializing in IK calculation and execution.

However, MoveIt only does the calculation and produces a path of waypoints by joint position.  This node executes the path on the hardware.

## Object Model

There are a few objects described in the diagram below that relate to both the logical abstraction used by MoveIt and the hardware backing it:

* `Joint`: This is how MoveIt communicates the movement of the arm.  Each Joint corresponds to an axis that can be independently controlled by MoveIt.  The position is measured by the rotation of the axis.
* `Motor`: This is how the HSI layer understands motion.  Motors don't explicitly have a position, they have a speed.
* `JointPositionMonitor`: This gets the position of the Joint in units MoveIt understands (radians).  This is typically accomplished by an encoder.

For most joints, there is a 1:1:1 correspondence from Joints to Motors to Encoders.  However, there is an outlier for the wrist of the arm, which has a 2:2:2 ratio.  The Encoders are 1:1 on the Joints, but the Motors are 2:2 on the Joints, since you need both motors to spin one axis.  To spin both axes, both motors needs to agree on how to spin both axes simultaneously, which necessitates the difference between a `DirectJointToMotor` controller and a `Differential` controller.

![Class Diagram of wr_control_drive_arm](Structure.png)
