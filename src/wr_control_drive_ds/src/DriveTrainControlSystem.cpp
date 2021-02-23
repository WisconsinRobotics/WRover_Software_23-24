#include "ros/ros.h"
#include "wr_drive_msgs/DriveTrainCmd.h"
#include "wroboclaw/Int16Pair.h"

#include <stdlib.h>
#include <stdio.h>

using namespace std;
float inp_vals[2];

void setDSPower_callback(const wr_drive_msgs::DriveTrainCmd::ConstPtr& msg){
	inp_vals[0] = msg->left_value;
	inp_vals[1] = msg->right_value;
}

int main(int argc, char** argv){
	
	ros::init(argc, argv, "DriveTrainControlSystem");

	ros::NodeHandle n;
	ros::NodeHandle n_priv;

	ros::Subscriber inTopic = n.subscribe("/control/drive_train_cmd", 1000, setDSPower_callback);

	ros::Publisher outTopic = n.advertise<wroboclaw::Int16Pair>("/hsi/drive_train/cmd", 1000);	// Not really DriveTrainCmd, have to find wherever Int16Pair is

	ros::Rate loop(50);

	while(ros::ok()){

		wroboclaw::Int16Pair output;

		// Run the calculations
		output.left = (int16_t)(inp_vals[0] * ((1<<15)-1));
		output.right = (int16_t)(inp_vals[1] * ((1<<15)-1));

		outTopic.publish(output);
	
		ros::spinOnce();

		loop.sleep();
	}
}
