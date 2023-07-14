# wr_control_drive_science

@defgroup wr_control_drive_science wr_control_drive_science

## Hardware Description (move me to wr_control_drive_science)

The science box contains:

* A claw attached to a vertical gantry (used for collecting soil)
* A turntable containing soil samples
  * The turntable contains the chemical tests for soil samples to determine the presence and kind of life in the soil
  * The turntable has a hole in it to allow the claw to pass through the box and collect soil
  * The precise positioning required by the turntable necessitates an encoder
* A vertically-actuated side attachment holds a soil moisture sensor and USB microscope
  * The timing requirements for the sensor require an Arduino to communicate with the sensor and cache its readings
  * USB microscope streams data back to the operators
