# wr_hsi_sensing

@defgroup wr_hsi_sensing wr_hsi_sensing
@brief Provides GPS and IMU data to the ROS network

This package contains nodes for the GPS sensor and IMU sensor on the WReaper board.  These sensors communicate directly with the hardware over the I2C bus.

The current IMU implementation doesn't actually use the IMU (due to a thermal failure at competition), but rather uses the movement of the GPS over time to derive a really coarse estimate of the heading of the rover.

## Launching

The `hw_test.launch` file starts both the IMU and GPS sensing.  Currently, since the IMU logic was not run at competition, only the GPS node is started.

# We're doing rtk now and it'll need a few things

Initialize the project: `./assemble.py init`
Make the venv so ros runs: `source setup.sh` (I think build does this)
Clean if broken: `./assemble.py clean`
Build the project: `./assemble.py build` (will take forever, installs dependencies too)

Check which tty exists: `ls /dev/tty*`
First we'll need access to a file so run: `sudo chmod a+rw /dev/ttyACM0`
Then run the node: `roslaunch wr_hsi_sensing rtk_test.launch`

To see all ros topics run: `rostopic list`
To get info of a topic run: `rostopic info /gps/fix`
To see all messages on a topic run: `rostopic echo /gps/fix`
