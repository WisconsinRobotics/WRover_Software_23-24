#!/usr/bin/env python3

import rospy
from wr_control_drive_science.srv import ScienceService
from wr_control_drive_science.srv import ScienceServiceRequest
from wr_control_drive_science.srv import ScienceServiceResponse


def main():
    '''
    Creates a new node called "service_client"
    '''
    rospy.init_node("science_client")

    '''
    Creates a new callable object for a service named "double_service" that is defined by the 
    "DoubleService" class.
    '''
    science_service = rospy.ServiceProxy("science_service", ScienceService)
    rospy.wait_for_service("science_service")

    rate = rospy.Rate(10)

    '''
    The funciton rospy.is_shutdown() checks if the module is still running or not. 
    '''
    while not rospy.is_shutdown():

        '''
        Here we use a try-except block to guard against any potential exceptions that might occur 
        when interacting with the service. 
        '''

        '''
        Calls the service whe user presses c
        '''
        
        if input("Enter c to print data: ") == "c":
            # rospy.loginfo("HEYYYYYYYY")
            response:ScienceServiceResponse = science_service()
            # rospy.loginfo(response)
            print("vol_water_reading: " + str(response.vol_water_reading))   
            print("temperature_reading: " + str(response.temperature_reading))
            print("conductivity_reading: " + str(response.conductivity_reading))   
            print()

      
        '''
        Waits for 100ms before resuming the program. This ensures that the loop runs at 10hz, 
        which was provided in the constructor.
        '''
        rate.sleep()

if __name__ == "__main__":
    main()