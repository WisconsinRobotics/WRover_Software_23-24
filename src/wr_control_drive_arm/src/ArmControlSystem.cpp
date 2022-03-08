/**
 * @file ArmControlSystem.cpp
 * @author Ben Nowotny
 * @brief The exeutable file to run the Arm Control Action Server
 * @date 2021-04-05
 */
#include "ros/ros.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <algorithm>
#include <csignal>
#include <string>
#include "SimpleJoint.hpp"
#include "DifferentialJoint.hpp"


/**
 * @brief Defines space for all ArmMotor references
 */
ArmMotor *motors[7];

/**
 * @brief Defines space for all Joint references
 */
AbstractJoint *joints[6];
/**
 * @brief The Joint State Publisher for MoveIt
 */
ros::Publisher jointStatePublisher;
/**
 * @brief Simplify the SimpleActionServer reference name
 */
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

/**
 * @brief converts a roll and pitch to joint motor positions
 * 
 * @param roll the roll of the joint in radians
 * @param pitch the pitch of the joint in radians
 * @return a 2D vector with motor positions in radians
 */
std::vector<double> convertJointSpacetoEncoderSpace(double roll, double pitch){
	std::vector<double> encoderSpace(2);

	//TODO: add matrix transformation, implement joint in mock.py
	encoderSpace[0] = roll;
	encoderSpace[1] = pitch;

	return encoderSpace;
}

/**
 * @brief sets a target position and pulls info from motor
 * 
 * @param names a vector with motor joint names
 * @param positions a vector with positions in radians
 * @param target the target radian value
 * @param motor a pointer to the motor
 * @return if the motor has reached its target
 */
bool configJointSetpoint(AbstractJoint* joint, int degreeIndex, std::vector<std::string>& names, std::vector<double>& positions, double target, float velocity){
	// Each motor should run to its respective target position at a fixed speed
	// TODO: this speed should be capped/dynamic to reflect the input joint velocity parameters
	// velMax = abs(*std::max_element(currTargetPosition.velocities.begin(), currTargetPosition.velocities.end(), [](double a, double b) {return abs(a)<abs(b);}));
	// float currPower = 0.1 * currTargetPosition.velocities[j]/velMax;
	// currPower = abs(velMax) <= 0.0001 ? 0.1 : currPower;
  std::cout << "config joint setup: " << target << std::endl;
  // std::cout << joint->getName() << ":" << motorIndex << " position: " << target;

	joint->configSetpoint(degreeIndex, target, 0);
	// Push the current motor name and position data to the Joint State data tracking list
	names.push_back(joint->getMotor(degreeIndex)->getMotorName());
	positions.push_back(joint->getMotor(degreeIndex)->getRads());
	// The position has only finished if every motor is STOPped
	return joint->getMotor(degreeIndex)->getMotorState() == MotorState::STOP;
}

/**
 * @brief Perform the given action as interpreted as moving the arm joints to specified positions
 * 
 * @param goal The goal state given
 * @param as The Action Server this is occuring on
 */
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {
  std::cout << "start exec: " << goal->trajectory.points.size() << std::endl;
  // For each point in the trajectory execution sequence...
  for(int i = 0; i < goal->trajectory.points.size(); i++){
    // Capture the current goal for easy reference
    trajectory_msgs::JointTrajectoryPoint currTargetPosition = goal->trajectory.points[i];

    // Track whether or not the current position is done
    bool hasPositionFinished = false;
    // Capture the names of the motors
    std::vector<std::string> names;
    // Capture the positions of the motors
    std::vector<double> positions;
    // Keep max loop rate at 50 Hz
    ros::Rate loop(200);

    // While the current position is not complete yet...
    while(!hasPositionFinished){
      // Assume the current action is done until proven otherwise
      hasPositionFinished = true;
      // Create the Joint State message for the current update cycle
      sensor_msgs::JointState js_msg;

      // Clear the list of motor names and position data
      names.clear();
      positions.clear();

      double velMax = 0;
      // For each joint specified in the currTargetPosition...
      int motorIndex = 0;
      int jointIndex = 0;
      AbstractJoint *joint;

      for(int j = 0; j < sizeof(joints); j += sizeof(joints[jointIndex-1])){

        joint = joints[jointIndex];

        for(int k = 0; k < joint->getDegreesOfFreedom(); k++){
          double targetPos = currTargetPosition.positions[motorIndex];

          std::cout<< "target pos: " << motorIndex << " " << targetPos <<std::endl;
          bool hasMotorFinished = configJointSetpoint(joint, motorIndex-jointIndex, names, positions, targetPos, 0);
          hasPositionFinished &= hasMotorFinished;
          motorIndex++;
          // DEBUGGING OUTPUT: Print each motor's name, radian position, encoder position, and power
          // std::cout<<joint->getMotor(k)->getMotorName()<<":"<<std::setw(30-motors[j]->getMotorName().length())<<motors[j]->getRads()<<std::endl;
          // std::cout<<std::setw(30)<<motors[j]->getEncoderCounts()<<std::endl;
          // std::cout<<std::setw(30)<<motors[j]->getPower()<<std::endl;
        }
        if (!joint->exectute()) {
          as->setAborted();
          return;
          // TODO: hand control back to driver
          // standard service empty stdsrvs empty
          // go into spin loop
        }
        jointIndex++;
      }
      
      // DEBUGGING OUTPUT: Print a divider line for cleanliness
      // std::cout<<velMax<<std::endl;
      std::cout<<"-----------------------"<<std::endl;
      // TODO: Make debugging output parameterized or pushed to the ROS output system to clean up output when desired
      
      // Set the name and position data for the Joint State Data message as tracked by the list of motor names and positions
      js_msg.name = names;
      js_msg.position = positions;
      // Publish the Joint State message
      jointStatePublisher.publish(js_msg);

      // Sleep until the next update cycle
      loop.sleep();
    }
  }

  //When all positions have been reached, set the current task as succeeded
  as->setSucceeded();
}

/**
 * @brief The main executable method of the node.  Starts the ROS node and the Action Server for processing Arm Control commands
 * 
 * @param argc The number of program arguments
 * @param argv The given program arguments
 * @return int The status code on exiting the program
 */
int main(int argc, char** argv)
{
  std::cout << "start main" << std::endl;
  // Initialize the current node as ArmControlSystem
  ros::init(argc, argv, "ArmControlSystem");
  // Create the NodeHandle to the current ROS node
  ros::NodeHandle n;

  // Initialize all motors with their MoveIt name, WRoboclaw initialization, and reference to the current node
  motors[0] = new ArmMotor("link1_joint", 0, 0, &n);
  motors[1] = new ArmMotor("link2_joint", 0, 1, &n);
  motors[2] = new ArmMotor("link3_joint", 1, 0, &n);
  motors[3] = new ArmMotor("link4_joint", 1, 1, &n);
  motors[4] = new ArmMotor("link5_joint", 2, 0, &n);
  motors[5] = new ArmMotor("link6_joint", 2, 1, &n);
  motors[6] = new ArmMotor("link7_joint", 3, 0, &n);
  std::cout << "init motors" << std::endl;

  // Initialize all Joints
  joints[0] = new SimpleJoint(motors[0], &n);
  joints[1] = new SimpleJoint(motors[1], &n);
  joints[2] = new SimpleJoint(motors[2], &n);
  joints[3] = new SimpleJoint(motors[3], &n);
  joints[4] = new SimpleJoint(motors[4], &n);
  DifferentialJoint* temp = new DifferentialJoint(motors[5], motors[6], &n);
  temp->configVelocityHandshake("/control/arm/5/roll", "/control/arm/5/pitch", "/control/arm/21/", "/control/arm/30/");
  joints[5] = temp;
  // joints[5] = new SimpleJoint(motors[5], &n);
  // joints[5]->configVelocityHandshake("/control/arm/5", "/control/arm/21");
  // joints[6] = new SimpleJoint(motors[6], &n);
  // joints[6]->configVelocityHandshake("/control/arm/6", "/control/arm/30");
  std::cout << "init joints" << std::endl;
  

  // Initialize the Joint State Data Publisher
  jointStatePublisher = n.advertise<sensor_msgs::JointState>("/control/arm_joint_states", 1000);

  // Initialize the Action Server
  Server server(n, "/arm_controller/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
  // Start the Action Server
  server.start();

  // signal(SIGINT, [](int signal)->void{ros::shutdown(); exit(1);});

  // ROS spin for communication with other nodes
  ros::spin();
  // Return 0 on exit (successful exit)

  // delete &motors[0];
  // delete &motors[1];
  // delete &motors[2];
  // delete &motors[3];
  // delete &motors[4];
  // delete &motors[5];
  // delete &motors[6];


  return 0;
}