# wr_hsi_sensing

@defgroup wr_hsi_sensing wr_hsi_sensing
@brief Provides GPS and IMU data to the ROS network

This package contains nodes for the GPS sensor and IMU sensor on the WReaper board.  These sensors communicate directly with the hardware over the I2C bus.

The current IMU implementation doesn't actually use the IMU (due to a thermal failure at competition), but rather uses the movement of the GPS over time to derive a really coarse estimate of the heading of the rover.

## Launching

The `hw_test.launch` file starts both the IMU and GPS sensing.  Currently, since the IMU logic was not run at competition, only the GPS node is started.
