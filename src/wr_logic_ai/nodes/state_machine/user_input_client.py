#!/usr/bin/env python


import rospy
from wr_logic_ai.srv import EmptySrv
# import keyboard

def wait_for_user_input_client():
    # print()
    rospy.wait_for_service('user_input_service')
    try:
        while True:
            if input("Enter c to continue:") == "c":
                wait_for_user = rospy.ServiceProxy('user_input_service', EmptySrv)
                wait_for_user()
                return
    except rospy.ServiceException as e:
        print(e)

if __name__ == "__main__":
    wait_for_user_input_client()