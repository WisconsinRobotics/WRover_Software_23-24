#!/usr/bin/env python

from wr_logic_ai.state_machine import NavStateMachine
from wr_logic_ai.coordinate_manager import CoordinateManager
import rospy

if __name__ == "__main__":
    rospy.init_node('nav_state_machine', anonymous=False)
    statemachine = NavStateMachine(CoordinateManager())
    rospy.spin()