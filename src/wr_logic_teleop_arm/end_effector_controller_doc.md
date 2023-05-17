# END EFFECTOR CONTROLLER
## EXPECTED BEHAVIORS
* When the A button is pressed (`std_msgs::Bool` on `/hci/arm/gamepad/button/a`), the claw will open (publish `std_msgs::Int16(32767)` to `/hsi/roboclaw/aux3/cmd/left`)
* When the B button is pressed (`std_msgs::Bool` on `/hci/arm/gamepad/button/b`), the claw will close (publish `std_msgs::Int16(-32768)` to `/hsi/roboclaw/aux3/cmd/left`)
* When neigther or both of the A and B buttons are pressed, the claw will not move (publish `std_msgs::Int16(0)` to `/hsi/roboclaw/aux3/cmd/left`)  
  
* When the Y button is pressed (`std_msgs::Bool` on `/hci/arm/gamepad/button/y)`, the solenoid will extend (write a "1" to the `sysfs` system, see below)
* When the Y button is not pressed, the solenoid will retract (write a "0" to the `sysfs` system, see below)

## SYSFS:
`sysfs` is a way of using filesystem-like objects to control Linux hardware.  The procedure to use a GPIO pin undewr this system is the following:
* Write the pin number to `/sys/class/gpio/export` (Our pin is "6") (If it's already exported, you don't have to do this)
* Wait for the file `/sys/class/gpio/gpio6` to become available (takes a second or two)
* Set the direction of the pin to "out" by writing to `/sys/class/gpio/gpio6/`direction
* To turn the pin on or off, wirte a "1" or "0" to `/sys/class/gpio/gpio6/value`, respectively
* To cleanup after the program is done, write the pin number back to `/sys/class/gpio/unexport`
Make sure you only open these files in write-only mode and flush the file write buffer after every write.

## GENRAL DESIGN IDEAS:
I recommend making a class to handle the claw open/close and another class to encapsulate the GPIO calls.  Then, in main, you can construct those objects and build the subscribers so they use those member functions as callbacks.  For the claw, make sure you only publish something when you hear an A or B message (*don't* publish continuously).  Ideally, all of the actions can happen in subscriber callbacks.  Once the ROS objects are set up, call `ros::spin()`, which will automatically process callbacks until ROS shuts down.  This means that once `ros::spin()` exits, you should start the clean-up procedure.