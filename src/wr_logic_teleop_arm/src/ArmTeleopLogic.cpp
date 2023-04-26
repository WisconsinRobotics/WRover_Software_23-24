#include "boost/function.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <atomic>
#include <boost/concept_check.hpp>
#include <cmath>
#include <cstdint>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <thread>

using Std_Bool = const std_msgs::BoolConstPtr &;
using Std_Float32 = const std_msgs::Float32ConstPtr &;

std::atomic_bool isNewPath{true};
const tf2::Quaternion WORLD_OFFSET{0, sin(M_PI / 2), 0, cos(M_PI / 2)};

auto updateTarget(float x_pos, float y_pos, float z_pos, tf2::Quaternion orientation, ros::Publisher &pub) -> void {
    geometry_msgs::PoseStamped p{};
    p.pose.position.x = x_pos;
    p.pose.position.y = y_pos;
    p.pose.position.z = z_pos;
    auto outOrientation = /* tf2::Quaternion(0,sin(M_PI/4), 0, cos(M_PI/4)) * */ orientation;
    p.pose.orientation = tf2::toMsg(outOrientation);
    p.header.frame_id = "base_link";
    pub.publish(p);
}

auto main(int argc, char **argv) -> int {
    ros::init(argc, argv, "ArmTeleopLogic");
    ros::AsyncSpinner spin(1);
    spin.start();
    ros::NodeHandle np("~");

    constexpr float PLANNING_TIME = 0.05;
    constexpr float CLOCK_RATE = 2;
    constexpr uint32_t MESSAGE_QUEUE_LENGTH = 1000;
    constexpr float TRIGGER_PRESSED = 0.5;
    constexpr float JOYSTICK_DEADBAND = 0.1;
    constexpr float MINIMUM_PATH_ACCURACY = 0.0;

    constexpr float STEP_X = 0.003;
    constexpr float STEP_Y = 0.003;
    constexpr float STEP_Z = 0.003;

    constexpr float HOME_X = 0.0;
    constexpr float HOME_Y = -0.7;
    constexpr float HOME_Z = 0.2;

    float accel = 1.0;
    float deaccel = 1.0;
    float step_mult = 1.0;
    float x_pos = HOME_X;
    float y_pos = HOME_Y;
    float z_pos = HOME_Z;

    const tf2::Quaternion homeOrientation{sin(M_PI / 4), 0, 0, cos(-M_PI / 4)};
    tf2::Quaternion orientation{homeOrientation};

    const tf2::Quaternion SPIN_X{sin(2 * M_PI / 1000), 0, 0, cos(2 * M_PI / 1000)};
    const tf2::Quaternion SPIN_Y{0, sin(2 * M_PI / 1000), 0, cos(2 * M_PI / 1000)};
    const tf2::Quaternion SPIN_Z{0, 0, sin(2 * M_PI / 1000), cos(2 * M_PI / 1000)};

    moveit::planning_interface::MoveGroupInterface move("arm");
    // move.setPlannerId("RRTStar");
    move.setPlanningTime(PLANNING_TIME);
    const moveit::core::JointModelGroup *joint_model_group = move.getCurrentState()->getJointModelGroup("arm");
    robot_state::RobotState start_state(*move.getCurrentState());
    moveit_visual_tools::MoveItVisualTools visual_tools("arm");

    ros::Rate loop{CLOCK_RATE};

    ros::Publisher nextTarget = np.advertise<geometry_msgs::PoseStamped>("/logic/arm_teleop/next_target",
                                                                         MESSAGE_QUEUE_LENGTH);

    ros::Publisher trajectoryPub = np.advertise<visualization_msgs::MarkerArray>("/logic/arm_teleop/trajectory",
                                                                                 MESSAGE_QUEUE_LENGTH);

    // y axis
    ros::Subscriber yAxis = np.subscribe("/hci/arm/gamepad/axis/stick_left_y",
                                         MESSAGE_QUEUE_LENGTH,
                                         static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
                                             if (abs(msg->data) >= JOYSTICK_DEADBAND) {
                                                 y_pos += msg->data * STEP_Y;
                                                 updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
                                             }
                                         }));

    // x axis
    ros::Subscriber xAxis = np.subscribe("/hci/arm/gamepad/axis/stick_left_x",
                                         MESSAGE_QUEUE_LENGTH,
                                         static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
                                             if (abs(msg->data) >= JOYSTICK_DEADBAND) {
                                                 x_pos += msg->data * STEP_X;
                                                 updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
                                             }
                                         }));

    // y up
    ros::Subscriber zUp = np.subscribe("/hci/arm/gamepad/button/trigger_left",
                                       MESSAGE_QUEUE_LENGTH,
                                       static_cast<boost::function<void(Std_Bool)>>([&](Std_Bool msg) -> void {
                                           if (msg->data) {
                                               z_pos += STEP_Y * step_mult;
                                               updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
                                           }
                                       }));

    // y down
    ros::Subscriber zDown = np.subscribe("/hci/arm/gamepad/button/trigger_right",
                                         MESSAGE_QUEUE_LENGTH,
                                         static_cast<boost::function<void(Std_Bool)>>([&](Std_Bool msg) -> void {
                                             if (msg->data) {
                                                 z_pos -= STEP_Y * step_mult;
                                                 updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
                                             }
                                         }));

    // roll
    ros::Subscriber roll = np.subscribe("/hci/arm/gamepad/button/pov_x",
                                          MESSAGE_QUEUE_LENGTH,
                                          static_cast<boost::function<void(Std_Bool)>>([&](Std_Bool msg) -> void {
                                              if (msg->data) {
                                                  orientation *= SPIN_Z * step_mult;
                                                  updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
                                              }
                                          }));

    // pitch
    ros::Subscriber pitch = np.subscribe("/hci/arm/gamepad/axis/pov_y",
                                         MESSAGE_QUEUE_LENGTH,
                                         static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
                                             if (abs(msg->data) >= 0.5) {
                                                 orientation *= (msg->data > 0 ? SPIN_Y : SPIN_Y.inverse());
                                                 updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
                                             }
                                         }));

    // yaw
    ros::Subscriber yaw = np.subscribe("/hci/arm/gamepad/axis/stick_right_x",
                                       MESSAGE_QUEUE_LENGTH,
                                       static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
                                           if (abs(msg->data) >= 0.5) {
                                               orientation *= (msg->data > 0 ? SPIN_X : SPIN_X.inverse());
                                               updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
                                           }
                                       }));

    // override path execution
    ros::Subscriber execPath = np.subscribe("/hci/arm/gamepad/button/start",
                                            MESSAGE_QUEUE_LENGTH,
                                            static_cast<boost::function<void(Std_Bool)>>([&](Std_Bool msg) -> void {
                                                if (msg->data) {
                                                    isNewPath.store(true);
                                                }
                                            }));

    // reset target/cancel path
    ros::Subscriber resetPose = np.subscribe("/hci/arm/gamepad/button/select",
                                             MESSAGE_QUEUE_LENGTH,
                                             static_cast<boost::function<void(Std_Bool)>>([&](Std_Bool msg) -> void {
                                                 if (msg->data) {
                                                     move.stop();
                                                 }
                                             }));

    ros::Subscriber home = np.subscribe("/hci/arm/gamepad/button/mode",
                                        MESSAGE_QUEUE_LENGTH,
                                        static_cast<boost::function<void(Std_Bool)>>([&](Std_Bool msg) -> void {
                                            if (msg->data) {
                                                x_pos = HOME_X;
                                                y_pos = HOME_Y;
                                                z_pos = HOME_Z;
                                                orientation = homeOrientation;
                                                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
                                            }
                                        }));

    while (ros::ok()) {

        updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
        if (!isNewPath.load()) {
            loop.sleep();
            continue;
        }

        // stop current path
        isNewPath.store(false);
        move.stop();

        // configure target pose
        geometry_msgs::PoseStamped p{};
        p.pose.position.x = x_pos;
        p.pose.position.y = y_pos;
        p.pose.position.z = z_pos;
        p.pose.orientation = tf2::toMsg(orientation);
        p.header.frame_id = "world";

        move.setPoseTarget(p);
        move.setStartStateToCurrentState();

        // plan and execute path
        move.asyncMove();

        while (ros::ok()) {
            updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);

            if (move.getMoveGroupClient().getState().isDone()) {
                std::cout << "[INFO] [" << ros::Time::now() << "]: path finished: " << move.getMoveGroupClient().getState().getText() << std::endl;
                break;
            }

            else if (isNewPath.load()) {
                std::cout << "[INFO] [" << ros::Time::now() << "]: path overridden" << std::endl;
                move.stop();
                break;
            }

            loop.sleep();
        }
    }

    return 0;
}