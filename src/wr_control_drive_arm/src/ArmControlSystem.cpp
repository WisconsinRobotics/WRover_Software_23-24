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
        sensor_msgs::JointState js_msg;

        names.clear();
        positions.clear();

        for(int j = 0; j < currTargetPosition.positions.size(); j++){
          motors[j]->runToTarget(currTargetPosition.positions[j], 0.1);
          hasPositionFinished &= motors[j]->getMotorState() == MotorState::STOP;
          names.push_back(motors[j]->getMotorName());
          positions.push_back(motors[j]->getRads());
          
          std::cout<<motors[j]->getMotorName()<<":"<<std::setw(30-motors[j]->getMotorName().length())<<motors[j]->getRads()<<std::endl;
          std::cout<<std::setw(30)<<motors[j]->getEncoderCounts()<<std::endl;
        }
        std::cout<<"-----------------------"<<std::endl;
        //TODO: Set js_msg
        js_msg.name = names;
        js_msg.position = positions;

        jointStatePublisher.publish(js_msg);
        loop.sleep();
      }
    }
    as->setSucceeded();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ArmControlSystem");
  ros::NodeHandle n;

  motors[0] = new ArmMotor("link1_joint", 0, 0, &n);
  motors[1] = new ArmMotor("link2_joint", 0, 1, &n);
  motors[2] = new ArmMotor("link3_joint", 1, 0, &n);
  motors[3] = new ArmMotor("link4_joint", 1, 1, &n);
  motors[4] = new ArmMotor("link5_joint", 2, 0, &n);
  motors[5] = new ArmMotor("link6_joint", 2, 1, &n);

  jointStatePublisher = n.advertise<sensor_msgs::JointState>("/control/arm_joint_states", 1000);
  Server server(n, "/arm_controller/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);

  server.start();
  ros::spin();
  return 0;
}