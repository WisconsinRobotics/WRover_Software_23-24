#include "ros/ros.h"
#include "wr_logic_teleop_ds/DriveTrainCmd.h"
#include "std_msgs/Bool.h"

#include <sstream>

int main(int argc, char** argv){

	ros::init(argc,argv,"TeleopDriveTrainLogic");

	ros::NodeHandle n;
	
	ros::Publisher driveCommand = n.advertise<wr_logic_teleop_ds::DriveTrainCmd>("TeleopDriveTrainCommand",1000);

	ros::Rate loop(50);

	while(ros::ok()){
	
		wr_logic_teleop_ds::DriveTrainCmd output;
		output.left_value = 1.0;
		output.right_value = -1.0;

		driveCommand.publish(output);

		ROS_INFO("(%f, %f)",output.left_value, output.right_value);

		ros::spinOnce();

		loop.sleep();
	
	}	

}
