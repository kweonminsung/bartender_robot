#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <string>
#include <vector>
#include <map>
#include <memory>

// DYNAMIXEL Control Table Addresses (XL430-W250 기준)
#define ADDR_OPERATING_MODE         11
#define ADDR_TORQUE_ENABLE          64
#define ADDR_POSITION_D_GAIN        80
#define ADDR_POSITION_P_GAIN        84
#define ADDR_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION       132
#define ADDR_PROFILE_VELOCITY       112

// Protocol version
#define PROTOCOL_VERSION            2.0

// Default settings
#define BAUDRATE                    115200
#define DEVICENAME                  "/dev/ttyUSB0"

// Position value range (0~4095 for 4096 positions per revolution)
#define DXL_MINIMUM_POSITION_VALUE  0
#define DXL_MAXIMUM_POSITION_VALUE  4095
#define DXL_MOVING_STATUS_THRESHOLD 20

class DynamixelController : public rclcpp::Node
{
public:
  DynamixelController();
  ~DynamixelController();

private:
  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  
  bool setupDynamixel();
  void shutdownDynamixel();
  
  bool setTorqueEnable(uint8_t id, bool enable);
  bool setGoalPosition(uint8_t id, int32_t position);
  int32_t getPresentPosition(uint8_t id);
  
  // Conversion helpers
  int32_t radianToPosition(double radian);
  double positionToRadian(int32_t position);

  // ROS communication
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;

  // Dynamixel SDK
  std::unique_ptr<dynamixel::PortHandler> port_handler_;
  std::unique_ptr<dynamixel::PacketHandler> packet_handler_;

  // Joint configuration
  std::vector<std::string> joint_names_;
  std::map<std::string, uint8_t> joint_id_map_;  // joint name -> dynamixel ID
  std::map<std::string, double> joint_positions_;
  
  // Parameters
  std::string device_name_;
  int baudrate_;
};
