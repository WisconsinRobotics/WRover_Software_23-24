#include "ros/ros.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include <thread>
#include "ArmMotor.hpp"

ArmMotor* motors[7];
ros::Publisher jointStatePublisher;
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {
    // for(int i = 0; i < goal->trajectory.points.size(); i++){
    //   trajectory_msgs::JointTrajectoryPoint currTargetPosition = goal->trajectory.points[i];
      
      
    // }
    motors[0]->setPower(0.01);
    int i = 0;
    ros::Rate sleeper(50);
    while(i++ < 100){
      // std::cout<<motors[0]->getEncoderCounts()<<std::endl;
      // std::cout<<motors[0]->getPower()<<std::endl;
      // std::cout<<std::endl;
      // std::this_thread::yield();
      sleeper.sleep();
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
  motors[6] = new ArmMotor("link7_joint", 3, 0, &n);

  jointStatePublisher = n.advertise<sensor_msgs::JointState>("/control/arm_joint_states", 1000);
  Server server(n, "/arm_controller/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);

  server.start();
  ros::spin();
  return 0;
}