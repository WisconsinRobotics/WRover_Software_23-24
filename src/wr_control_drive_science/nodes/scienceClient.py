#!/usr/bin/env python3

import rospy
from wr_control_drive_science.srv import ScienceService
from wr_control_drive_science.srv import ScienceServiceRequest
from wr_control_drive_science.srv import ScienceServiceResponse

from wr_control_drive_science.msg import LightMsg
import csv
import math

CSV_FILE = csv_file = 'lightDataSample'
count = 0
class RecordData:
    
    def __init__(self):
        global count
        '''
        Creates a new callable object for a service named "science_service" that is defined by the 
        "ScienceService" class. The data will be stored in files inside: ~/.ros
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
            Calls the service when user presses c
            '''
            userInput = input("Enter c to print data \nEnter a to start recording data \nEnter b to stop recording data: ")
            if userInput == "c":
                # rospy.loginfo("HEYYYYYYYY")
                response:ScienceServiceResponse = science_service()
                # rospy.loginfo(response)
                print("vol_water_reading: " + str(response.vol_water_reading))   
                print("temperature_reading: " + str(response.temperature_reading))
                print("conductivity_reading: " + str(response.conductivity_reading))   
                print()
            elif userInput == "a":
                count = count + 1
                rospy.loginfo("Writing sample " +str(count) + " of light data into CSV \n\n")
                # Write the values to the CSV file
                self.file = open(str(CSV_FILE) + str(count) + ".csv", mode='w', newline='')
                writer = csv.writer(self.file)
                # Write the header
                writer.writerow(['Time(s)', 'LightDNA(Lux)', 'LightProtein(Lux)'])

                self.start_time = rospy.get_time()

                subToLight = rospy.Subscriber("light_data", LightMsg , self.callback)
                
            elif userInput == "b":
                rospy.loginfo("Stopped recording data")
                subToLight.unregister()
                self.file.close()
            
            '''
            Waits for 100ms before resuming the program. This ensures that the loop runs at 10hz, 
            which was provided in the constructor.
            '''
            rate.sleep()
    def callback(self, data):
        # Write the values to the CSV file        
        writer = csv.writer(self.file)
        # Write the data
        writer.writerow([math.floor(rospy.get_time() - self.start_time), data.LightDNA, data.LightProtein])

if __name__ == "__main__":
    rospy.init_node("science_client")
    RecordData()
    rospy.spin()