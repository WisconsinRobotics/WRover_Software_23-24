#include "ros/ros.h"
#include "wr_drive_msgs/DriveTrainCmd.h"
#include "wr_drive_msgs/CamMastCmd.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <array>
#include "Watchdog.hpp"

#include <stdlib.h>
#include <stdio.h>

//Variables to hold the speeds and speed ratios for each side
std::array<float, 2> speedRatio{0.0, 0.0};
std::array<float, 2> speedRaw{0.0, 0.0};
//Holds the constant speed ratios (provided by parameters)
constexpr std::array<float, 4> DEFAULT_SPEED_RATIOS{0.25, 0.5, 0.75, 1.0};
std::array<float, 4> speedRatioValues{DEFAULT_SPEED_RATIOS};
//The camera mast speed value
float speedCamMast = 0.0;
//Cache for msg reception: [0] is left, [1] is right
std::array<bool, 2> msgCache{false, false};

//Default number of messages to cache for any publisher or subscriber
constexpr std::uint32_t MESSAGE_CACHE_SIZE = 10;

typedef std_msgs::Bool::ConstPtr Std_Bool;
typedef std_msgs::Float32::ConstPtr Std_Float32;

/*
 * Drive train message
 */
//Define the output message
wr_drive_msgs::DriveTrainCmd output;

//////////////////////////////////////////////////////////////////
//			BUTTON CALLBACKS			//
//								//
//	These update the speed ratios for their respective side //
// when their button is pressed.				//
//////////////////////////////////////////////////////////////////

//Generic Button Callback
void genCallback(const Std_Bool &msg, int ind1, int ind2){
	if(static_cast<bool>(msg->data)) speedRatio.at(ind1) = speedRatioValues.at(ind2);
}

//Left Drive Joystick
boost::function<void(const Std_Bool&)>  L_S3_cb = [](const Std_Bool &msg)->void{genCallback(msg, 0, 3);};
boost::function<void(const Std_Bool&)>  L_S2_cb = [](const Std_Bool &msg)->void{genCallback(msg, 0, 2);};
boost::function<void(const Std_Bool&)>  L_S1_cb = [](const Std_Bool &msg)->void{genCallback(msg, 0, 1);};
boost::function<void(const Std_Bool&)>  L_S0_cb = [](const Std_Bool &msg)->void{genCallback(msg, 0, 0);};

//Right Drive Joystick
boost::function<void(const Std_Bool&)>  R_S0_cb = [](const Std_Bool &msg)->void{genCallback(msg, 1, 0);};
boost::function<void(const Std_Bool&)>  R_S1_cb = [](const Std_Bool &msg)->void{genCallback(msg, 1, 1);};
boost::function<void(const Std_Bool&)>  R_S2_cb = [](const Std_Bool &msg)->void{genCallback(msg, 1, 2);};
boost::function<void(const Std_Bool&)>  R_S3_cb = [](const Std_Bool &msg)->void{genCallback(msg, 1, 3);};

void cachePublish();
ros::Publisher driveCommand;
ros::Publisher camCommand;

//////////////////////////////////////////////////////////////////
//		      JOYSTICK CALLBACKS		      //
//							      //
//      These update the raw speed for their respective side as //
// the joysticks are moved.					//
//////////////////////////////////////////////////////////////////

//Left Drive Joystick

void djL_axY_callback(const Std_Float32& msg){
	output.left_value = msg->data*speedRatio[0];
	msgCache[0] = true;
	cachePublish();
}

//Right Drive Joystick

void djR_axY_callback(const Std_Float32& msg){
	output.right_value = msg->data*speedRatio[1];
	msgCache[1] = true;
	cachePublish();
}

void djR_camMast_callback(const Std_Float32& msg){
	speedCamMast = msg->data;

	/*
	 * Camera mast message
	 */
	wr_drive_msgs::CamMastCmd cam_cmd;
	cam_cmd.turn_speed = speedCamMast;
	camCommand.publish(cam_cmd);
}

//////////////////////////////////////////
//			HELPER METHODS				//
//////////////////////////////////////////

void cachePublish() {
	// if both a left and right msg have been received
	if (msgCache[0] && msgCache[1]) {
		driveCommand.publish(output); // publish output values
		msgCache[0] = false; // reset the msg cache
		msgCache[1] = false;
	}
}

//Main Method

int main(int argc, char** argv){

	//ROS initialization
	ros::init(argc,argv,"TeleopDriveTrainLogic");

	//Handler to the current node
	ros::NodeHandle n;
	//Handler for private namespace
	ros::NodeHandle nh("~");

	//Get Custom Speed Parameters
	nh.getParam("speed_step1", speedRatioValues.at(0));
	nh.getParam("speed_step2", speedRatioValues.at(1));
	nh.getParam("speed_step3", speedRatioValues.at(2));
	nh.getParam("speed_step4", speedRatioValues.at(3));
	
	//Publisher for output data
	driveCommand = n.advertise<wr_drive_msgs::DriveTrainCmd>("/control/drive_system/cmd", MESSAGE_CACHE_SIZE);
	camCommand = n.advertise<wr_drive_msgs::CamMastCmd>("/control/camera/cam_mast_cmd", MESSAGE_CACHE_SIZE);
	
	//Assign the button callbacks to their respective topics
	auto s1 = n.subscribe("/logic/drive_system/joystick_left/button/3", MESSAGE_CACHE_SIZE, L_S2_cb);
	auto s2 = n.subscribe("/logic/drive_system/joystick_left/button/4", MESSAGE_CACHE_SIZE, L_S1_cb);
	auto s3 = n.subscribe("/logic/drive_system/joystick_left/button/5", MESSAGE_CACHE_SIZE, L_S3_cb);
	auto s4 = n.subscribe("/logic/drive_system/joystick_left/button/6", MESSAGE_CACHE_SIZE, L_S0_cb);
	auto s5 = n.subscribe("/logic/drive_system/joystick_right/button/3", MESSAGE_CACHE_SIZE, R_S1_cb);
	auto s6 = n.subscribe("/logic/drive_system/joystick_right/button/4", MESSAGE_CACHE_SIZE, R_S2_cb);
	auto s7 = n.subscribe("/logic/drive_system/joystick_right/button/5", MESSAGE_CACHE_SIZE, R_S0_cb);
	auto s8 = n.subscribe("/logic/drive_system/joystick_right/button/6", MESSAGE_CACHE_SIZE, R_S3_cb);

	//Assign the joystick callbacks to their respective topics
	auto sL = n. subscribe("/logic/drive_system/joystick_left/axis/stick_y", MESSAGE_CACHE_SIZE, djL_axY_callback);
	auto sR = n. subscribe("/logic/drive_system/joystick_right/axis/stick_y", MESSAGE_CACHE_SIZE, djR_axY_callback);

	//Subscriber for camera mast control
	auto sCamMast = n.subscribe("/logic/drive_system/joystick_right/axis/pov_x", MESSAGE_CACHE_SIZE, djR_camMast_callback);

	//ROS Update cycler
	ros::spin();
}
