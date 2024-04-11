/**
 * @defgroup wr_control_drive_ds_drive Drive Speed Translation for Freefall
 * @brief Translates logical drive speeds to hardware power values
 * @ingroup wr_control_drive_ds_freefall
 * @{
 */

/**
 * @file
 * @author Ben Nowotny
 */

#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "wr_drive_msgs/DriveTrainCmd.h"
#include "wroboclaw/Int16Pair.h"

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

    // Create a publisher to the output topic for WRoboClaw duty cycle numbers for command 34
    auto outTopic{nHandle.advertise<wroboclaw::Int16Pair>("/hsi/roboclaw/drive/cmd", MESSAGE_CACHE_SIZE)};

    // Create a subscriber to the input topic for power values
    auto inTopic{
        nHandle.subscribe("/control/drive_system/cmd", MESSAGE_CACHE_SIZE,
                          static_cast<boost::function<void(const wr_drive_msgs::DriveTrainCmd::ConstPtr &)>>(
                              [&outTopic](const wr_drive_msgs::DriveTrainCmd::ConstPtr &msg) {
                                  wroboclaw::Int16Pair outMsg;
                                  outMsg.left = static_cast<std::int16_t>(msg->left_value * INT16_MAX);
                                  outMsg.right = static_cast<std::int16_t>(msg->right_value * INT16_MAX);
                                  outTopic.publish(outMsg);
                              }))};

    ros::spin();

    return EXIT_SUCCESS;
}

/// @}
