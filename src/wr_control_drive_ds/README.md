# wr_control_drive_ds

@defgroup wr_control_drive_ds wr_control_drive_ds

This package takes high-level motor speeds for the drive system and translates those into speeds needed by the drivetrains.

## History and Purpose

This used to be (slighly) more involved with the old Roboclaw controllers, since they wanted a signed 16-bit number to set the speed of the motor (think: duty cycle).  The logical speed values are on the interval \[-1,1\], so this node did the translation.

It also inverts the right drivetrain speed since if both motors spin the same way with positive power and they are facing in opposite directions, applying positive power will make one drive 'forward' and the other drive 'backward'.  This allows the logic layer to assume that giving a motor positive power will always drive it 'forward'.

In light of switching to Falcon500 motors, the scaling is handled by a lower level driver.  The right-side inversion still happens in this node.
