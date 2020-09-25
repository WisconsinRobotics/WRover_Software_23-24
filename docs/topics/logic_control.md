# Logic/Control Transsection

* `/control/example: std_msgs/Float32`
  
  This topic allows information to flow from the high-level logic layer down to the control systems layer.

* `/logic/example: std_msgs/Float32`
  
  This topic allows information to flow from the control systems layer up to the high-level logic layer.

* `/control/drive_train_cmd: wr_drive_msgs/DriveTrainCmd`

  This topic allows the logic layer to send a pair of real numbers to the drivetrain as power commands.  Format: [Left Power, Right Power].
