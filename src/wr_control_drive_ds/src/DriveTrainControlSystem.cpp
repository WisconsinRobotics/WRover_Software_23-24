#include "ros/ros.h"
#include "wr_drive_msgs/DriveTrainCmd.h"
#include "wroboclaw/Int16Pair.h"

#include <stdlib.h>
#include <stdio.h>
#include <array>

using namespace std;
// Store the input abstract power values
std::array<float, 2> inp_vals{0.0, 0.0};

constexpr std::uint32_t MESSAGE_CACHE_SIZE = 10;

// Main program
int main(int argc, char** argv){
	
	// Initialize the node in ROS
	ros::init(argc, argv, "DriveTrainControlSystem");

	// Create a node handle for the current node to subscribe to/advertise topics
	ros::NodeHandle n;

	// Create a publisher to the output topic for WRoboClaw duty cycle numbers for command 34
	ros::Publisher outTopic = n.advertise<wroboclaw::Int16Pair>("/hsi/roboclaw/drive/cmd", MESSAGE_CACHE_SIZE);

	// Create a subscriber to the input topic for power values
	ros::Subscriber inTopic = n.subscribe("/control/drive_system/cmd", MESSAGE_CACHE_SIZE, 
	static_cast<boost::function<void(const wr_drive_msgs::DriveTrainCmd::ConstPtr&)>>(
		[&outTopic](const wr_drive_msgs::DriveTrainCmd::ConstPtr& msg){
			wroboclaw::Int16Pair outMsg;
			outMsg.left = static_cast<std::int16_t>(msg->left_value * INT16_MAX);
			outMsg.right = static_cast<std::uint16_t>(-msg->right_value * INT16_MAX);
			outTopic.publish(outMsg);
		}
	));

	ros::spin();
}
