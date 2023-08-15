# wr_control_drive_ds

@defgroup wr_control_drive_ds wr_control_drive_ds
@brief Translates logical drive speeds into hardware-usable drive speeds and controls the camera mast

This package takes high-level motor speeds for the drive system and translates those into speeds needed by the drivetrains.  It also controls the camera mast

## Drive Translation History and Purpose

This translates the drive command message into separate left and right hardware power messages.

It also inverts the right drivetrain speed since if both motors spin the same way with positive power and they are facing in opposite directions, applying positive power will make one drive 'forward' and the other drive 'backward'.  This allows the logic layer to assume that giving a side of the drivetrain positive power will always drive it 'forward'.

## Camera Mast Control

This listens to logical rotation speeds for the camera mast and outputs hardware speeds for the camera mast.  This still does the Roboclaw conversion.

## Roboclaw Conversion

This used to be (slighly) more involved with the old Roboclaw controllers, since they wanted a signed 16-bit number to set the speed of the motor (think: duty cycle).  The logical speed values are on the interval \[-1,1\], so this node did the translation for both the drive motors and the camera mast.

In light of switching to Falcon500 motors, the scaling for the drive system is handled by a lower level driver.  The camera mast still uses a Roboclaw controller and still needs the conversion.
