#include "BNO085_hal.hpp"

#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include <cstdlib>
#include <iostream>

static constexpr uint32_t QUEUE_SIZE = 1;

const bool debug = true;
BNO085 sensor(debug);

void sensor_setup() {
    // if (sensor.set_sensor_config(SH2_MAGNETIC_FIELD_UNCALIBRATED) != SH2_OK) {
    //     ROS_FATAL("FAILED TO ENABLE MAGNETIC FIELD");
    // }

    // if (sensor.set_sensor_config(SH2_MAGNETIC_FIELD_CALIBRATED) != SH2_OK) {
    // ROS_FATAL("FAILED TO ENABLE MAGNETIC FIELD CALIBRATED");
    // }

    if (sensor.set_sensor_config(SH2_GAME_ROTATION_VECTOR) != SH2_OK) {
        ROS_FATAL("FAILED TO ENABLE IMU GAME ROTATION VECTOR");
    }
}

auto main(int argc, char **argv) -> int {

    // Initialize the node in ROS
    ros::init(argc, argv, "BNO085");

    // Create a node handle for the current node to subscribe to/advertise topics
    ros::NodeHandle nHandle;

    ros::Rate rate(10);

    // Create publishers
    auto pub_x{nHandle.advertise<std_msgs::Float64>("/mag_x", QUEUE_SIZE)};
    auto pub_y{nHandle.advertise<std_msgs::Float64>("/mag_y", QUEUE_SIZE)};
    auto pub_z{nHandle.advertise<std_msgs::Float64>("/mag_z", QUEUE_SIZE)};
    auto pub_acc{nHandle.advertise<std_msgs::Int16>("/mag_acc", QUEUE_SIZE)};

    auto pub_heading{nHandle.advertise<std_msgs::Float64>("/heading_data", QUEUE_SIZE)};

    // Initalize IMU
    // nHandle.getParam("imu/debug", debug);
    if (!sensor.begin()) {
        ROS_FATAL("FAILED TO INITIALIZE IMU");
    }

    sensor_setup();

    if (!sensor.tare(true, SH2_TARE_BASIS_GAMING_ROTATION_VECTOR)) {
        ROS_WARN("FAILED TO TARE IMU GAME ROTATION VECTOR");
    } else {
        if (!sensor.persist_tare()) {
            ROS_WARN("IMU TARE WILL CHANGE ON NEXT RESET");
        }
    }
    sensor.tare(false, SH2_TARE_BASIS_GAMING_ROTATION_VECTOR);

    std_msgs::Float64 mag_x, mag_y, mag_z, heading;
    std_msgs::Int16 mag_acc;
    while (ros::ok()) {
        if (sensor.get_reset()) {
            ROS_INFO("IMU SENSOR RESET %d", sensor.prod_ids.entry[0].resetCause);
            sensor_setup();
        }

        if (sensor.get_sensor_event()) {

            // if (sensor.sensor_value.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED) {
            //     mag_x.data = sensor.get_mag_x();
            //     pub_x.publish(mag_x);

            //     mag_y.data = sensor.get_mag_y();
            //     pub_y.publish(mag_y);

            //     mag_z.data = sensor.get_mag_z();
            //     pub_z.publish(mag_z);

            //     mag_acc.data = static_cast<int16_t>(sensor.get_accuracy());
            //     pub_acc.publish(mag_acc);
            // }

            if (sensor.sensor_value.sensorId == SH2_GAME_ROTATION_VECTOR) {
                // Convert Euler angle to 0 to 360
                heading.data = (sensor.get_yaw() + M_PI) / M_PI * 180;
                pub_heading.publish(heading);
            }
        }

        rate.sleep();
    }

    return EXIT_SUCCESS;
}
