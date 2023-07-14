/**
 * @file TeleopScienceControl.cpp
 * @author Ben Nowotny
 * @brief Deprecated science control system
 *
 */

/**
 * @addtogroup wr_control_drive_science
 * @{
 * @defgroup
 */

#include "ros/assert.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/timer.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt32.h"
#include <array>
#include <fcntl.h>
#include <string>
#include <termios.h>

constexpr std::uint32_t MESSAGE_CACHE_SIZE = 10;
constexpr int SAMPLES_PER_READING = 3;
constexpr float TIMER_CALLBACK_DURATION = 5;
constexpr int NUM_CHARS_READ = SAMPLES_PER_READING * sizeof(float);

//NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
int fileHandle;
//NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::array<ros::Publisher, SAMPLES_PER_READING> sensorPublishers;

void moistureCallback(const ros::TimerEvent &timerEvent) {
    std::array<float, SAMPLES_PER_READING> buf{};
    /* Flush anything already in the serial buffer */
    tcflush(fileHandle, TCIFLUSH);
    /* read up to 128 bytes from the fd */
    long readStatus = read(fileHandle, buf.data(), NUM_CHARS_READ);
    //NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-pro-type-vararg, hicpp-no-array-decay, hicpp-no-assembler, hicpp-vararg)
    ROS_ASSERT_MSG(readStatus != -1, "Read failed");
    std_msgs::Float32 temp;
    for (int i = 0; i < SAMPLES_PER_READING; i++) {
        temp.data = buf.at(i);
        sensorPublishers.at(i).publish(temp);
    }
}

void setupFileHandle(const std::string &fileName, int baud);

auto main(int argc, char **argv) -> int {
    ros::init(argc, argv, "Science Teleop Control");

    ros::NodeHandle node;
    ros::NodeHandle privateNode{"~"};
    std::string file{};
    int baudRate = 0;
    privateNode.getParam("fileLoc", file);
    privateNode.getParam("baudRate", baudRate);

    setupFileHandle(file, baudRate);

    for (int i = 0; i < SAMPLES_PER_READING; i++) {
        using namespace std::string_literals;
        sensorPublishers.at(i) = node.advertise<std_msgs::Float32>("control/science/moisture"s + std::to_string(i), MESSAGE_CACHE_SIZE);
    }

    ros::Timer timer = node.createTimer(ros::Duration(TIMER_CALLBACK_DURATION), moistureCallback);

    ros::Publisher screwLiftPow = node.advertise<std_msgs::Int16>("/hsi/roboclaw/aux0/cmd/left", MESSAGE_CACHE_SIZE);
    ros::Publisher turnTablePow = node.advertise<std_msgs::Int16>("/hsi/roboclaw/aux0/cmd/right", MESSAGE_CACHE_SIZE);
    ros::Publisher linearActuatorPow = node.advertise<std_msgs::Int16>("/hsi/roboclaw/aux1/cmd/left", MESSAGE_CACHE_SIZE);
    ros::Publisher clawPow = node.advertise<std_msgs::Int16>("/hsi/roboclaw/aux1/cmd/right", MESSAGE_CACHE_SIZE);
    ros::Publisher turnTableEncoder = node.advertise<std_msgs::Float64>("pid/turnTable", MESSAGE_CACHE_SIZE);

    ros::Subscriber encoderSub = node.subscribe("/hsi/roboclaw/aux0/enc/right", MESSAGE_CACHE_SIZE,
                                                static_cast<boost::function<void(const std_msgs::UInt32::ConstPtr &)>>(
                                                    [&turnTableEncoder](const std_msgs::UInt32::ConstPtr &msg) {
                                                        std_msgs::Float64 out;
                                                        out.data = msg->data;
                                                        turnTableEncoder.publish(out);
                                                    }));
    ros::Subscriber screwLiftSub = node.subscribe("logic/science/screwLift", MESSAGE_CACHE_SIZE,
                                                  static_cast<boost::function<void(const std_msgs::Float32::ConstPtr &)>>(
                                                      [&screwLiftPow](const std_msgs::Float32::ConstPtr &msg) {
                                                          std_msgs::Int16 out;
                                                          out.data = static_cast<short>(msg->data * INT16_MAX);
                                                          screwLiftPow.publish(out);
                                                      }));
    ros::Subscriber turnTableSub = node.subscribe("pid/turnTablePow", MESSAGE_CACHE_SIZE,
                                                  static_cast<boost::function<void(const std_msgs::Float64::ConstPtr &)>>(
                                                      [&turnTablePow](const std_msgs::Float64::ConstPtr &msg) {
                                                          std_msgs::Int16 out;
                                                          out.data = static_cast<short>(msg->data * INT16_MAX);
                                                          turnTablePow.publish(out);
                                                      }));

    ros::Subscriber linearActuatorSub = node.subscribe("logic/science/linearActuator", MESSAGE_CACHE_SIZE,
                                                       static_cast<boost::function<void(const std_msgs::Float32::ConstPtr &)>>(
                                                           [&linearActuatorPow](const std_msgs::Float32::ConstPtr &msg) {
                                                               std_msgs::Int16 out;
                                                               out.data = static_cast<short>(msg->data * INT16_MAX);
                                                               linearActuatorPow.publish(out);
                                                           }));
    ros::Subscriber clawSub = node.subscribe("logic/science/claw", MESSAGE_CACHE_SIZE,
                                             static_cast<boost::function<void(const std_msgs::Float32::ConstPtr &)>>(
                                                 [&clawPow](const std_msgs::Float32::ConstPtr &msg) {
                                                     std_msgs::Int16 out;
                                                     out.data = static_cast<short>(msg->data * INT16_MAX);
                                                     clawPow.publish(out);
                                                 }));

    ros::spin();
}

template <typename T>
constexpr void setFlag(T &ref, unsigned int value) {
    ref |= value;
}

template <typename T>
constexpr void unsetFlag(T &ref, unsigned int value) {
    ref &= ~value;
}

void setupFileHandle(const std::string &fileName, int baud) {
    //NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg, hicpp-vararg, hicpp-signed-bitwise)
    fileHandle = open(fileName.c_str(), O_RDWR | O_NOCTTY);
    //NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-pro-type-vararg, hicpp-no-array-decay, hicpp-no-assembler, hicpp-vararg)
    ROS_ASSERT_MSG(fileHandle != -1, "File %s does not exist", fileName.c_str());
    /* Set up the control structure */
    termios toptions{};

    /* Get currently set options for the tty */
    tcgetattr(fileHandle, &toptions);

    /* Set custom options */
    cfsetispeed(&toptions, baud);
    /* 8 bits, no parity, no stop bits */
    unsetFlag(toptions.c_cflag, PARENB);
    unsetFlag(toptions.c_cflag, CSTOPB);
    unsetFlag(toptions.c_cflag, CSIZE);
    setFlag(toptions.c_cflag, CS8);

    /* no hardware flow control */
    unsetFlag(toptions.c_cflag, CRTSCTS);

    /* enable receiver, ignore status lines */
    setFlag(toptions.c_cflag, CREAD);
    setFlag(toptions.c_cflag, CLOCAL);

    /* disable input/output flow control, disable restart chars */
    unsetFlag(toptions.c_iflag, IXON);
    unsetFlag(toptions.c_iflag, IXOFF);
    unsetFlag(toptions.c_iflag, IXANY);

    /* disable canonical input, disable echo,
    disable visually erase chars,
    disable terminal-generated signals */
    unsetFlag(toptions.c_lflag, ICANON);
    unsetFlag(toptions.c_lflag, ECHO);
    unsetFlag(toptions.c_lflag, ECHOE);
    unsetFlag(toptions.c_lflag, ISIG);

    /* disable output processing */
    unsetFlag(toptions.c_oflag, OPOST);

    /* wait for 12 characters to come in before read returns */
    /* WARNING! THIS CAUSES THE read() TO BLOCK UNTIL ALL */
    /* CHARACTERS HAVE COME IN! */
    toptions.c_cc[VMIN] = NUM_CHARS_READ;
    /* no minimum time to wait before read returns */
    toptions.c_cc[VTIME] = 0;
    /* avoid hangup */
    unsetFlag(toptions.c_cflag, HUPCL);
    /* commit the options */
    tcsetattr(fileHandle, TCSANOW, &toptions);
}