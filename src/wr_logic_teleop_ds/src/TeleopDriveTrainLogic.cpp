#include "ros/ros.h"
#include "wr_logic_teleop_ds/DriveTrainCmd.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

#include <sstream>
#include <stdlib.h>
#include <stdio.h>

//Variables to hold the speeds and speed ratios for each side
float speedRatio[] = {0.0, 0.0};
float speedRaw[] = {0.0, 0.0};
//Holds the constant speed ratios (provided by parameters)
float SPEED_RATIO_VALUES[4];

//////////////////////////////////////////////////////////////////
//			BUTTON CALLBACKS			//
//								//
//	These update the speed ratios for their respective side //
// when their button is pressed.				//
//////////////////////////////////////////////////////////////////

//Left Drive Joystick

void djL_btn5_callback(const std_msgs::Bool::ConstPtr& msg){
        if(msg->data) speedRatio[0] = SPEED_RATIO_VALUES[0];
}

void djL_btn3_callback(const std_msgs::Bool::ConstPtr& msg){
        if(msg->data) speedRatio[0] = SPEED_RATIO_VALUES[1];
}

void djL_btn4_callback(const std_msgs::Bool::ConstPtr& msg){
        if(msg->data) speedRatio[0] = SPEED_RATIO_VALUES[2];
}

void djL_btn6_callback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data) speedRatio[0] = SPEED_RATIO_VALUES[3];
}

//Right Drive Joystick

void djR_btn5_callback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data) speedRatio[1] = SPEED_RATIO_VALUES[0];
}

void djR_btn3_callback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data) speedRatio[1] = SPEED_RATIO_VALUES[1];
}

void djR_btn4_callback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data) speedRatio[1] = SPEED_RATIO_VALUES[2];
}

void djR_btn6_callback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data) speedRatio[1] = SPEED_RATIO_VALUES[3];
}

//////////////////////////////////////////////////////////////////
//                      JOYSTICK CALLBACKS                      //
//                                                              //
//      These update the raw speed for their respective side as //
// the joysticks are moved.					//
//////////////////////////////////////////////////////////////////

//Left Drive Joystick

void djL_axY_callback(const std_msgs::Float32::ConstPtr& msg){
	speedRaw[0] = msg->data;
}

//Right Drive Joystick

void djR_axY_callback(const std_msgs::Float32::ConstPtr& msg){
	speedRaw[1] = msg->data;
}

//Main Method

int main(int argc, char** argv){
	
	//Speed Ratio constants MUST be provided.
	//If there aren't enough arguments, terminate.
	if(argc != 5) return 0;

	//Convert the parameters to floats and assign them to the constants array
	for(int i = 1; i < 5; i++) SPEED_RATIO_VALUES[i-1] = strtof(argv[i],NULL);
	
	//ROS initialization
	ros::init(argc,argv,"TeleopDriveTrainLogic");

	//Handler to the current node
	ros::NodeHandle n;
	
	//Publisher for output data to the drivetrain
	ros::Publisher driveCommand = n.advertise<wr_logic_teleop_ds::DriveTrainCmd>("/control/drive_train_cmd", 1000);
	
	//Loop Rate - 50 Hz
	ros::Rate loop(50);
	
	//Set up dummy subscribers for input data
	ros::Subscriber s1, s2, s3, s4, s5, s6, s7, s8, sL, sR;
	
	//Assign the button callbacks to their respective topics
	s1 = n.subscribe("/logic/drive_joystick_left/button/3", 1000, djL_btn3_callback);
	s2 = n.subscribe("/logic/drive_joystick_left/button/4", 1000, djL_btn4_callback);
	s3 = n.subscribe("/logic/drive_joystick_left/button/5", 1000, djL_btn5_callback);
        s4 = n.subscribe("/logic/drive_joystick_left/button/6", 1000, djL_btn6_callback);
	s5 = n.subscribe("/logic/drive_joystick_right/button/3", 1000, djR_btn3_callback);
        s6 = n.subscribe("/logic/drive_joystick_right/button/4", 1000, djR_btn4_callback);
	s7 = n.subscribe("/logic/drive_joystick_right/button/5", 1000, djR_btn5_callback);
        s8 = n.subscribe("/logic/drive_joystick_right/button/6", 1000, djR_btn6_callback);

	//Assign the joystick callbacks to their respective topics
	sL = n. subscribe("/logic/drive_joystick_left/axis/y", 1000, djL_axY_callback);
	sR = n. subscribe("/logic/drive_joystick_right/axis/y", 1000, djR_axY_callback);

	//ROS Main loop
	while(ros::ok()){
	
		//Define the output message
		wr_logic_teleop_ds::DriveTrainCmd output;
		
		//Set the left and right values to be the product of respective raw speeds and speed ratios
		output.left_value = speedRaw[0]*speedRatio[0];
		output.right_value = speedRaw[1]*speedRatio[1];

		//Publish the output message
		driveCommand.publish(output);

		//Print the message to ROS INFO for logging
		ROS_INFO("(%f, %f)",output.left_value, output.right_value);

		//Trigger ROS Update cycle
		ros::spinOnce();

		//Sleep until next cycle
		loop.sleep();
	
	}	

}
