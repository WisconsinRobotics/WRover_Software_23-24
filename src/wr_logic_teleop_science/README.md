# wr_logic_teleop_science

@defgroup wr_logic_teleop_science wr_logic_teleop_science

This package is a deprecated, ideolized software system for the high-level logic of controlling the Science Box.  

## Hardware Description (move me to wr_control_drive_science)

The science box contains:

* A claw attached to a vertical gantry (used for collecting soil)
* A turntable containing soil samples
  * The turntable contains the chemical tests for soil samples to determine the presence and kind of life in the soil
  * The turntable has a hole in it to allow the claw to pass through the box and collect soil
  * The precise positioning required by the turntable necessitates an encoder
* A vertically-actuated side attachment holds a soil moisture sensor and USB microscope
  * The timing requirements for the sensor require an Arduino to communicate with the sensor and cache its readings
  * Continue later

## Controls Layout

* POV hat: x-y axis control over a gantry with a soil collection claw on it  
* Right Stick Y Axis: "claw" control  
* Left/Right Shoulder Keys: Index left/right on the turntable through a PID controller  

### Controls Note

This control scheme is ideolized and deprecated due to having no matching hardware implmentation.  Talk to the science team members to get a better idea for what they're current goals are, what hardware they want to use, and the timeline they have in mind.  
