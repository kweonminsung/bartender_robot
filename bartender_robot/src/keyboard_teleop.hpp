#pragma once

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <string>
#include <vector>
#include <map>
#include <thread>
#include <atomic>

class KeyboardJointTeleop : public rclcpp::Node
{
public:
  KeyboardJointTeleop();
  ~KeyboardJointTeleop();

  void stop();

private:
  void keyboardLoop();
  void publishJointCommand();

  std::vector<std::string> joints_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_pub_;
  std::map<std::string, double> positions_;

  double step_;
  std::string controller_name_;

  std::thread kb_thread_;
  std::atomic<bool> running_;
};
