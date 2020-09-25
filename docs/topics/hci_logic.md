# HCI/Logic Transsection

* `/logic/example: std_msgs/String`
  
  This topic allows information to flow from the HCI layer down to the high-level logic layer.

* `/hci/example: std_msgs/Float32`
  
  This topic allows information to flow from the high-level logic layer up to the HSI layer

* `/logic/drive_joystick_(left|right)/axis/(x|y): std_msgs/Float32`

  This topic publishes x- or y-coordinate of the drive joystick's position to the logic layer (Range: [-1, 1]).

* `/logic/drive_joystick_(left|right)/button/(1-12): std_msgs/Bool`

  This topic publishes the status of buttons 1-12 on the drive joysticks to the logic layer.
