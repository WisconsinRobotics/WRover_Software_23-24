/**
* @defgroup wr_control_drive_ds_drive Drive Speed Translation
* @brief Translates logical drive speeds to hardware power values
* @ingroup wr_control_drive_ds
* @{
*/

/**
* @file
* @author Ben Nowotny
*/

#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/node_handle.h"
#include "std_msgs/Float64.h"
#include "wr_drive_msgs/DriveTrainCmd.h"
#include <cstdlib>

/**
 * @brief The main executable of the node
 * 
 * @param argc Number of program arguments
 * @param argv The program arguments
 * @return int The return code of the program
 */
auto main(int argc, char **argv) -> int {

  // Number of messages to queue in publishing/subscribing
  static constexpr std::uint32_t MESSAGE_CACHE_SIZE = 10;

  // Initialize the node in ROS
  ros::init(argc, argv, "DriveTrainControlSystem");

  // Create a node handle for the current node to subscribe to/advertise topics
  ros::NodeHandle nHandle;

  // Create a publisher to the output topic for WRoboClaw duty cycle numbers for
  // command 34
  auto outLeft {nHandle.advertise<std_msgs::Float64>("/left/power", MESSAGE_CACHE_SIZE)};

  auto outRight {nHandle.advertise<std_msgs::Float64>("/right/power", MESSAGE_CACHE_SIZE)};

  // Create a subscriber to the input topic for power values

  const auto republishSpeed{
    [&outLeft, &outRight](const wr_drive_msgs::DriveTrainCmd::ConstPtr &msg) {
      
                        std_msgs::Float64 left;
                        std_msgs::Float64 right;

                        left.data = msg->left_value;
                        right.data = -msg->right_value;

                        outLeft.publish(left);
                        outRight.publish(right);
                      }
  };

  auto inTopic{
    nHandle.subscribe<wr_drive_msgs::DriveTrainCmd>(
      "/control/drive_system/cmd", 
      MESSAGE_CACHE_SIZE, 
      republishSpeed)};

  ros::spin();

  return EXIT_SUCCESS;
}

/// @}
