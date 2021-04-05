#include "ros/ros.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include "ArmMotor.hpp"

// Define space for 6 ArmMotors
ArmMotor *motors[6];
// Define the Joint State Data Publisher
ros::Publisher jointStatePublisher;
// Simplify the SimpleActionServer typename
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

/**
 * @brief Perform the given action as interpreted as moving the arm joints to specified positions
 * 
 * @param goal The goal state given
 * @param as The Action Server this is occuring on
 */
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {
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
      ros::Rate loop(50);

      // While the current position is not complete yet...
      while(!hasPositionFinished){
        // Assume the current action is done until proven otherwise
        hasPositionFinished = true;
        // Create the Joint State message for the current update cycle
        sensor_msgs::JointState js_msg;

        // Clear the list of motor names and position data
        names.clear();
        positions.clear();

        // For each motor specified in the currTargetPosition...
        for(int j = 0; j < currTargetPosition.positions.size(); j++){
          // Each motor should run to its respective target position at a fixed speed
          // TODO: this speed should be capped/dynamic to reflect the input joint velocity parameters
          motors[j]->runToTarget(currTargetPosition.positions[j], 0.1);
          // The position has only finished if every motor is STOPped
          hasPositionFinished &= motors[j]->getMotorState() == MotorState::STOP;
          // Push the current motor name and position data to the Joint State data tracking list
          names.push_back(motors[j]->getMotorName());
          positions.push_back(motors[j]->getRads());
          
          // DEBUGGING OUTPUT: Print each motor's name, radian position, and encoder position
          std::cout<<motors[j]->getMotorName()<<":"<<std::setw(30-motors[j]->getMotorName().length())<<motors[j]->getRads()<<std::endl;
          std::cout<<std::setw(30)<<motors[j]->getEncoderCounts()<<std::endl;
        }
        // DEBUGGING OUTPUT: Print a divider line for cleanliness
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

  // Initialize the Joint State Data Publisher
  jointStatePublisher = n.advertise<sensor_msgs::JointState>("/control/arm_joint_states", 1000);

  // Initialize the Action Server
  Server server(n, "/arm_controller/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
  // Start the Action Server
  server.start();

  // ROS spin for communication with other nodes
  ros::spin();
  // Return 0 on exit (successful exit)
  return 0;
}