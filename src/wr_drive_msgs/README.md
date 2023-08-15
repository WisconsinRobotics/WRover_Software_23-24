# wr_drive_msgs

@defgroup wr_drive_msgs wr_drive_msgs
@brief Builds the drive command ROS messages

This package contains the definitions for the drive command messages.  That's it.  Other packages that depend on this message should list it as a dependency in their `CMakeLists.txt` file in the `find_package(catkin REQUIRED COMPONENTS ...)` clause.

## Design

This message is used to define the communications for logic-layer nodes on the drive speeds.  This message is based on directly controlling a tank-drive drivetrain, where the left and right power are used in conjunction to drive and steer.  See the below diagram for details:

![Tank Drive Controls](TankDriveControls.png)

The left and right power levels are always required in the same message.  This implies that anyone publishing to the hardware drive topic is *owning* the drive system, exclusively.  Arbitration between multiple nodes that want to own drive system controls should multiplex via a mechaism like [ROS `topic_tools` mux](http://wiki.ros.org/topic_tools/mux).
