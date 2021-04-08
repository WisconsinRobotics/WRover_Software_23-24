#include "ros/ros.h"
#include "wr_drive_msgs/DriveTrainCmd.h"
#include "wroboclaw/Int16Pair.h"

#include <stdlib.h>
#include <stdio.h>

using namespace std;
// Store the input abstract power values
float inp_vals[2];

// Reads messages from the /control/drive_train_cmd topic as abstract power values (Float32 on [-1, 1])
void setDSPower_callback(const wr_drive_msgs::DriveTrainCmd::ConstPtr& msg){
	// Capture each power value in the input array
	inp_vals[0] = msg->left_value;
	inp_vals[1] = msg->right_value;
}

// Main program
int main(int argc, char** argv){
	
	// Initialize the node in ROS
	ros::init(argc, argv, "DriveTrainControlSystem");

	// Create a node handle for the current node to subscribe to/advertise topics
	ros::NodeHandle n;

	// Create a subscriber to the input topic for power values
	ros::Subscriber inTopic = n.subscribe("/control/drive_system/cmd", 1000, setDSPower_callback);

	// Create a publisher to the output topic for WRoboClaw duty cycle numbers for command 34
	ros::Publisher outTopic = n.advertise<wroboclaw::Int16Pair>("/hsi/roboclaw/drive/cmd", 1000);

	// Create the processing rate (50 Hz)
	ros::Rate loop(50);

	// Create the (resuable) output message
	wroboclaw::Int16Pair output;

	// While ROS is active...
	while(ros::ok()){

		// Convert input power values to duty cycle numbers by scaling to a signed 16 bit number.
		output.left = (int16_t)(inp_vals[0] * INT16_MAX);
		output.right = (int16_t)(inp_vals[1] * INT16_MAX);

		// Publish the current message
		outTopic.publish(output);
	
		// ROS Spin
		ros::spinOnce();

		// Sleep until the next cycle
		loop.sleep();
	}
}
