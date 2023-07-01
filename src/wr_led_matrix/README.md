# wr_led_matrix

@defgroup wr_led_matrix wr_led_matrix
A package to control an LED panel

This package offers a service to allow users to set the color of the LED panel required for the Autonomous Navigation challenge in URC.  The service takes a color in RGB format.  No feedback is supplied; it is assumed that if the service returns without a ROS `ServiceException` that the request succeeded.

## Launching

This package offers a launch file at `launch/led_matrix.launch`.  This supports a mocking parameter (currently via the `WROVER_HW` environment variable) to disable actually writing to hardware, just providing the service instead.
