/**
 * @file ArmControlSystem.cpp
 * @author Ben Nowotny
 * @brief The exeutable file to run the Arm Control Action Server
 * @date 2021-04-05
 */
#include "XmlRpcValue.h"

#include "ros/ros.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <algorithm>
#include <csignal>
#include <string>
#include <std_srvs/Trigger.h>
#include "SimpleJoint.hpp"
#include "DifferentialJoint.hpp"
#include "ros/console.h"

using XmlRpc::XmlRpcValue;

/**
 * @brief Defines space for all ArmMotor references
 */
const int numMotors = 6;
ArmMotor *motors[numMotors];

/**
 * @brief Defines space for all Joint references
 */
const int numJoints = 5;
AbstractJoint *joints[numJoints];
/**
 * @brief The Joint State Publisher for MoveIt
 */
ros::Publisher jointStatePublisher;
/**
 * @brief Simplify the SimpleActionServer reference name
 */
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
/**
 * @brief The service server for enabling IK
 */
ros::ServiceServer enableServiceServer;
/**
 * @brief The status of IK program
 */
std::atomic_bool IKEnabled{true};
/**
 * @brief The service client for disabling IK
 */
ros::ServiceClient enableServiceClient;

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
bool configJointSetpoint(AbstractJoint* joint, int degreeIndex, double target, float velocity){
	// Each motor should run to its respective target position at a fixed speed
	// TODO: this speed should be capped/dynamic to reflect the input joint velocity parameters
	// velMax = abs(*std::max_element(currTargetPosition.velocities.begin(), currTargetPosition.velocities.end(), [](double a, double b) {return abs(a)<abs(b);}));
	// float currPower = 0.1 * currTargetPosition.velocities[j]/velMax;
	// currPower = abs(velMax) <= 0.0001 ? 0.1 : currPower;
  std::cout << "config joint setup: " << degreeIndex << " " << target << std::endl;
	joint->configSetpoint(degreeIndex, target, 0);
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
  if (!IKEnabled) {
    as->setAborted();
    ROS_WARN("IK is disabled");
    return;
  }

  std::cout << "start exec: " << goal->trajectory.points.size() << std::endl;
  // For each point in the trajectory execution sequence...
  for(int i = 0; i < goal->trajectory.points.size(); i++){
    std::cout << "PID to trajectory point " << i << "/" <<  goal->trajectory.points.size() << std::endl;
    // Capture the current goal for easy reference
    trajectory_msgs::JointTrajectoryPoint currTargetPosition = goal->trajectory.points[i];

    // Track whether or not the current position is done
    bool hasPositionFinished = false;
    // Keep max loop rate at 50 Hz
    ros::Rate loop(200);

    // While the current position is not complete yet...
    while(!hasPositionFinished){
      // Assume the current action is done until proven otherwise
      hasPositionFinished = true;
      // Create the Joint State message for the current update cycle
      double velMax = abs(*std::max_element(
        currTargetPosition.velocities.begin(),
        currTargetPosition.velocities.end(),
        [](double a, double b) { return abs(a)<abs(b); }
      ));
      int motorIndex = 0;
      int jointIndex = 0;
      AbstractJoint *joint;
      // For each joint specified in the currTargetPosition...

      for(int j = 0; j < sizeof(joints); j += sizeof(joints[jointIndex-1])){

        joint = joints[jointIndex];

        for(int k = 0; k < joint->getDegreesOfFreedom(); k++){
        // Each motor should run to its respective target position at a fixed speed
        // TODO: this speed should be capped/dynamic to reflect the input joint velocity parameters
          double targetPos = currTargetPosition.positions[motorIndex];
          float currPower = 0.1 * currTargetPosition.velocities[j]/velMax;

          bool hasMotorFinished = configJointSetpoint(joint, motorIndex-jointIndex, targetPos, currPower);
          hasPositionFinished &= hasMotorFinished;
          motorIndex++;
          // DEBUGGING OUTPUT: Print each motor's name, radian position, encoder position, and power
          // std::cout<<joint->getMotor(k)->getMotorName()<<":"<<std::setw(30-motors[j]->getMotorName().length())<<motors[j]->getRads()<<std::endl;
          // std::cout<<std::setw(30)<<motors[j]->getEncoderCounts()<<std::endl;
          // std::cout<<std::setw(30)<<motors[j]->getPower()<<std::endl;
        }
        if (!joint->exectute()) {
          IKEnabled = false;
          std_srvs::Trigger srv;
          if (enableServiceClient.call(srv)) {
            ROS_WARN("%s", (std::string{"PLACEHOLDER_NAME: "} + srv.response.message).data());
          } else {
            ROS_WARN("Error: failed to call service PLACEHOLDER_NAME");
          }
          return;
          // TODO: hand control back to driver
          // standard service empty stdsrvs empty
          // go into spin loop
        }
        jointIndex++;
      }

      // DEBUGGING OUTPUT: Print a divider line for cleanliness
      std::cout<<"-----------------------"<<std::endl;
      // TODO: Make debugging output parameterized or pushed to the ROS output system to clean up output when desired

      // Sleep until the next update cycle
      loop.sleep();
    }
  }

  //When all positions have been reached, set the current task as succeeded

  for(int i = 0; i < numMotors; i++){
    motors[i]->setPower(0.f);
  }
  
  as->setSucceeded();
}

/**
 * @brief publishes the arm's position
 */
void publish(const ros::TimerEvent &event){
  std::vector<std::string> names;
  std::vector<double> positions;
  sensor_msgs::JointState js_msg;

  for (int i = 0; i < numMotors; i++){
    names.push_back(motors[i]->getMotorName());
    positions.push_back(motors[i]->getRads());
  }

  positions[4] = positions[5] + positions[4] / 2;

  js_msg.name = names;
  js_msg.position = positions;
  // Publish the Joint State message
  jointStatePublisher.publish(js_msg);
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
  ros::NodeHandle pn{"~"};

  XmlRpcValue encParams;
  pn.getParam("encoder_parameters", encParams);

  // Initialize all motors with their MoveIt name, WRoboclaw initialization, and reference to the current node
  motors[0] = new ArmMotor("elbow", 1, 0, static_cast<int>(encParams[0]["counts_per_rotation"]), static_cast<int>(encParams[0]["offset"]), n);
  motors[1] = new ArmMotor("forearm_roll", 1, 1, static_cast<int>(encParams[1]["counts_per_rotation"]), static_cast<int>(encParams[1]["offset"]), n);
  motors[2] = new ArmMotor("shoulder", 0, 1, static_cast<int>(encParams[2]["counts_per_rotation"]), static_cast<int>(encParams[2]["offset"]), n);
  motors[3] = new ArmMotor("turntable", 0, 0, static_cast<int>(encParams[3]["counts_per_rotation"]), static_cast<int>(encParams[3]["offset"]), n);
  motors[4] = new ArmMotor("wrist_pitch", 2, 0, static_cast<int>(encParams[4]["counts_per_rotation"]), static_cast<int>(encParams[4]["offset"]), n);
  motors[5] = new ArmMotor("wrist_roll", 2, 1, static_cast<int>(encParams[5]["counts_per_rotation"]), static_cast<int>(encParams[5]["offset"]), n);
  std::cout << "init motors" << std::endl;

  // Initialize all Joints
  joints[0] = new SimpleJoint(motors[0], &n);
  joints[1] = new SimpleJoint(motors[1], &n);
  joints[2] = new SimpleJoint(motors[2], &n);
  joints[3] = new SimpleJoint(motors[3], &n);
  joints[4] = new SimpleJoint(motors[4], &n);
  DifferentialJoint* temp = new DifferentialJoint(motors[4], motors[5], &n);
  temp->configVelocityHandshake("/control/arm/5/roll", "/control/arm/5/pitch", "/control/arm/20/", "/control/arm/21/");

  std::cout << "init joints" << std::endl;
  joints[5] = temp;
  
  // Initialize the Joint State Data Publisher
  jointStatePublisher = n.advertise<sensor_msgs::JointState>("/control/arm_joint_states", 1000);

  // Initialize the Action Server
  Server server(n, "/arm_controller/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
  // Start the Action Server
  server.start();
  std::cout << "server started" << std::endl;

  ros::Timer timer = n.createTimer(ros::Duration(1.0 / 50.0), publish);

  enableServiceServer = n.advertiseService("start_IK", 
    static_cast<boost::function<bool(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)>>(
      [](std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)->bool{
        IKEnabled = true;
        res.message = "Arm IK Enabled";
        res.success = true;
        return true;
      }
    ));

  enableServiceClient = n.serviceClient<std_srvs::Trigger>("PLACEHOLDER_NAME");
  
  std::cout << "entering ROS spin..." << std::endl;
  // ROS spin for communication with other nodes
  ros::spin();
  // Return 0 on exit (successful exit)
  return 0;
}
