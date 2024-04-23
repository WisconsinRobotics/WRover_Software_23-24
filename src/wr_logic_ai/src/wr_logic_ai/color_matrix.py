import rospy
import time

from wr_led_matrix.srv import (
    led_matrix as LEDMatrix,
    led_matrixRequest as LEDMatrixRequest,
)

## LED matrix color for when the rover is navigating towards the target using autonomous navigation
COLOR_AUTONOMOUS = LEDMatrixRequest(RED=0, GREEN=0, BLUE=255)
## LED matrix color for when the rover has reached its target
COLOR_COMPLETE = LEDMatrixRequest(RED=0, GREEN=255, BLUE=0)
## LED matrix color for when the rover has encountered an error while executing autonomous navigation
COLOR_ERROR = LEDMatrixRequest(RED=255, GREEN=0, BLUE=0)
## Initial LED matrix color
COLOR_NONE = LEDMatrixRequest(RED=0, GREEN=0, BLUE=0)


def set_matrix_color(color: LEDMatrixRequest) -> None:
    """Helper function for setting the LED matrix color

    @param color The color to set the LED matrix to
    """
    matrix_srv = rospy.ServiceProxy("/led_matrix", LEDMatrix)
    matrix_srv.wait_for_service()
    matrix_srv.call(COLOR_NONE)
    time.sleep(0.5)
    matrix_srv.call(color)
