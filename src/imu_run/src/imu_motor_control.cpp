#include <algorithm>
#include <cctype>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#define MOTOR_WRITE_COMMAND 0x01
#define MOTOR_WORK_MODE_ADDRESS 0x0F
#define MOTOR_ENABLE_ADDRESS 0x10
#define MOTOR_TARGET_SPEED_ADDRESS 0x09

#define MOTOR_VELOCITY_MODE_VALUE 0x00000003
#define MOTOR_ENABLE_VALUE 0x00000001
#define MOTOR_INTER_COMMAND_DELAY_MS 5

#define MOTOR_PULSES_PER_REVOLUTION 65536.0
#define MOTOR_SECONDS_PER_MINUTE 60.0
#define MOTOR1_CAN_ID "015"
#define MOTOR2_CAN_ID "016"
#define MOTOR1_DEFAULT_TARGET_RPM 0.0
#define MOTOR2_DEFAULT_TARGET_RPM 0.0
#define MOTOR_IMU_CMD_VEL_TOPIC "/imu_cmd_vel"

namespace
{
struct CommandStep
{
  std::string name;
  std::vector<int> frame;
};

std::vector<int> buildWriteFrame(uint8_t address, int32_t value);
bool sendFrame(const std::string& can_interface,
               const std::string& can_id,
               const CommandStep& step);
int32_t rpmToProtocolSpeed(double rpm);

class ImuMotorController
{
public:
  ImuMotorController(const std::string& name,
                     const std::string& can_interface,
                     const std::string& can_id)
    : name_(name), can_interface_(can_interface), can_id_(can_id)
  {
  }

  bool initialize()
  {
    const std::vector<CommandStep> steps = {
      {name_ + " | Step 1 - set velocity mode",
       buildWriteFrame(MOTOR_WORK_MODE_ADDRESS, MOTOR_VELOCITY_MODE_VALUE)},
      {name_ + " | Step 2 - enable motor",
       buildWriteFrame(MOTOR_ENABLE_ADDRESS, MOTOR_ENABLE_VALUE)}
    };

    for (std::size_t i = 0; i < steps.size(); ++i) {
      if (!sendFrame(can_interface_, can_id_, steps[i])) {
        return false;
      }

      if (i + 1 < steps.size()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(MOTOR_INTER_COMMAND_DELAY_MS));
      }
    }

    return true;
  }

  void sendTargetSpeed(double rpm)
  {
    const int32_t target_speed_value = rpmToProtocolSpeed(rpm);
    ROS_INFO_STREAM(name_ << " target speed " << rpm
                    << " rpm -> protocol value " << target_speed_value);

    const CommandStep step = {
      name_ + " | Step 3 - send target speed",
      buildWriteFrame(MOTOR_TARGET_SPEED_ADDRESS, target_speed_value)
    };

    sendFrame(can_interface_, can_id_, step);
  }

private:
  std::string name_;
  std::string can_interface_;
  std::string can_id_;
};

bool isHexString(const std::string& value)
{
  if (value.empty()) {
    return false;
  }

  return std::all_of(value.begin(), value.end(), [](unsigned char ch) {
    return std::isxdigit(ch) != 0;
  });
}

bool isSafeInterfaceName(const std::string& value)
{
  if (value.empty()) {
    return false;
  }

  return std::all_of(value.begin(), value.end(), [](unsigned char ch) {
    return std::isalnum(ch) != 0 || ch == '_' || ch == '-';
  });
}

std::vector<int> int32ToBigEndianBytes(int32_t value)
{
  const uint32_t raw_value = static_cast<uint32_t>(value);
  return {
    static_cast<int>((raw_value >> 24) & 0xFF),
    static_cast<int>((raw_value >> 16) & 0xFF),
    static_cast<int>((raw_value >> 8) & 0xFF),
    static_cast<int>(raw_value & 0xFF)
  };
}

std::vector<int> buildWriteFrame(uint8_t address, int32_t value)
{
  const std::vector<int> data_bytes = int32ToBigEndianBytes(value);
  return {
    static_cast<int>(MOTOR_WRITE_COMMAND),
    static_cast<int>(address),
    data_bytes[0],
    data_bytes[1],
    data_bytes[2],
    data_bytes[3]
  };
}

std::string formatPayload(const std::vector<int>& data_bytes)
{
  std::ostringstream payload;
  payload << std::uppercase << std::hex << std::setfill('0');

  for (int byte_value : data_bytes) {
    payload << std::setw(2) << byte_value;
  }

  return payload.str();
}

std::string buildCanSendCommand(const std::string& can_interface,
                                const std::string& can_id,
                                const std::vector<int>& data_bytes)
{
  return "cansend " + can_interface + " " + can_id + "#" + formatPayload(data_bytes);
}

std::string formatStandardCanId(const std::string& can_id)
{
  unsigned int can_id_value = 0;
  std::stringstream parser;
  parser << std::hex << can_id;
  parser >> can_id_value;

  std::ostringstream formatted_id;
  formatted_id << std::uppercase << std::hex << std::setfill('0') << std::setw(3) << can_id_value;
  return formatted_id.str();
}

bool sendFrame(const std::string& can_interface,
               const std::string& can_id,
               const CommandStep& step)
{
  const std::string command = buildCanSendCommand(can_interface, can_id, step.frame);
  ROS_INFO_STREAM(step.name << ": " << command);

  const std::string full_command = command + " 2>&1";
  std::array<char, 256> buffer{};
  std::string command_output;

  FILE* pipe = popen(full_command.c_str(), "r");
  if (pipe == nullptr) {
    ROS_ERROR_STREAM("Failed to start command for step [" << step.name << "]: " << command);
    return false;
  }

  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    command_output += buffer.data();
  }

  const int status = pclose(pipe);
  const int exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : status;
  if (status != 0) {
    ROS_ERROR_STREAM("Failed at step [" << step.name << "]: " << command
                     << " | exit_code=" << exit_code
                     << " | output=" << (command_output.empty() ? "<empty>" : command_output));
    return false;
  }

  return true;
}

int32_t rpmToProtocolSpeed(double rpm)
{
  return static_cast<int32_t>(rpm * MOTOR_PULSES_PER_REVOLUTION / MOTOR_SECONDS_PER_MINUTE);
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_motor_control");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string can_interface = "can0";
  std::string motor1_can_id = MOTOR1_CAN_ID;
  std::string motor2_can_id = MOTOR2_CAN_ID;
  double motor1_target_rpm = MOTOR1_DEFAULT_TARGET_RPM;
  double motor2_target_rpm = MOTOR2_DEFAULT_TARGET_RPM;
  std::string imu_cmd_vel_topic = MOTOR_IMU_CMD_VEL_TOPIC;

  private_nh.param<std::string>("can_interface", can_interface, can_interface);
  private_nh.param<std::string>("motor1_can_id", motor1_can_id, motor1_can_id);
  private_nh.param<std::string>("motor2_can_id", motor2_can_id, motor2_can_id);
  private_nh.param<double>("motor1_target_rpm", motor1_target_rpm, motor1_target_rpm);
  private_nh.param<double>("motor2_target_rpm", motor2_target_rpm, motor2_target_rpm);
  private_nh.param<std::string>("imu_cmd_vel_topic", imu_cmd_vel_topic, imu_cmd_vel_topic);

  if (!isSafeInterfaceName(can_interface)) {
    ROS_ERROR("Parameter ~can_interface contains unsupported characters.");
    return 1;
  }

  if (!isHexString(motor1_can_id) || motor1_can_id.size() > 3) {
    ROS_ERROR("Parameter ~motor1_can_id must be a hexadecimal standard CAN ID in range 000-7FF.");
    return 1;
  }

  if (!isHexString(motor2_can_id) || motor2_can_id.size() > 3) {
    ROS_ERROR("Parameter ~motor2_can_id must be a hexadecimal standard CAN ID in range 000-7FF.");
    return 1;
  }

  const std::string motor1_can_id_hex = formatStandardCanId(motor1_can_id);
  const std::string motor2_can_id_hex = formatStandardCanId(motor2_can_id);

  ROS_INFO_STREAM("left-right motor hex id " << motor1_can_id << " -> CAN id " << motor1_can_id_hex);
  ROS_INFO_STREAM("front-back motor hex id " << motor2_can_id << " -> CAN id " << motor2_can_id_hex);

  ImuMotorController motor1_controller("motor_015_left_right", can_interface, motor1_can_id_hex);
  ImuMotorController motor2_controller("motor_016_front_back", can_interface, motor2_can_id_hex);

  if (!motor1_controller.initialize()) {
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(MOTOR_INTER_COMMAND_DELAY_MS));

  if (!motor2_controller.initialize()) {
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(MOTOR_INTER_COMMAND_DELAY_MS));
  motor1_controller.sendTargetSpeed(motor1_target_rpm);
  std::this_thread::sleep_for(std::chrono::milliseconds(MOTOR_INTER_COMMAND_DELAY_MS));
  motor2_controller.sendTargetSpeed(motor2_target_rpm);

  ROS_INFO_STREAM("Waiting for gimbal command updates on topic: " << imu_cmd_vel_topic);
  ROS_INFO("imu_cmd_vel mapping: angular.z -> motor 015 (left-right), angular.y -> motor 016 (front-back)");

  ros::Subscriber imu_cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(
    imu_cmd_vel_topic,
    10,
    [&motor1_controller, &motor2_controller](const geometry_msgs::Twist::ConstPtr& msg) {
      motor1_controller.sendTargetSpeed(msg->angular.z);
      std::this_thread::sleep_for(std::chrono::milliseconds(MOTOR_INTER_COMMAND_DELAY_MS));
      motor2_controller.sendTargetSpeed(msg->angular.y);
    });

  ros::spin();
  return 0;
}
