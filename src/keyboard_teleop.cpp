#include "keyboard_teleop.hpp"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <iostream>
#include <chrono>

// Helper RAII to set terminal to raw (non-canonical) mode
struct TermSettings {
  struct termios oldt{};
  bool valid{false};
  TermSettings() {
    if (tcgetattr(STDIN_FILENO, &oldt) == 0) {
      struct termios newt = oldt;
      newt.c_lflag &= ~(ICANON | ECHO);
      newt.c_cc[VMIN] = 0;
      newt.c_cc[VTIME] = 0;
      if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) == 0) {
        valid = true;
      }
    }
  }
  ~TermSettings() {
    if (valid) tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }
};

KeyboardJointTeleop::KeyboardJointTeleop()
: Node("keyboard_joint_teleop"), running_(true)
{
  // declare parameters
  this->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
  this->declare_parameter<double>("step", 0.2);

  this->declare_parameter<std::string>("controller_name", "joint_trajectory_controller");

  joints_ = this->get_parameter("joints").as_string_array();
  step_ = this->get_parameter("step").as_double();
  controller_name_ = this->get_parameter("controller_name").as_string();

  if (joints_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No joints provided via parameter 'joints'.");
  }

  // Initialize positions to 0
  for (const auto &j : joints_) {
    positions_[j] = 0.0;
    RCLCPP_INFO(this->get_logger(), "Joint '%s' initialized", j.c_str());
  }

  // Create publisher for joint trajectory
  std::string topic = "/" + controller_name_ + "/joint_trajectory";
  trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(topic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to topic: %s", topic.c_str());

  // Debug: Print all joint names
  RCLCPP_INFO(this->get_logger(), "=== DEBUG: Configured Joints ===");
  for (size_t i = 0; i < joints_.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "  [%zu] %s", i, joints_[i].c_str());
  }
  RCLCPP_INFO(this->get_logger(), "================================");

  // start keyboard thread
  kb_thread_ = std::thread(&KeyboardJointTeleop::keyboardLoop, this);
}

KeyboardJointTeleop::~KeyboardJointTeleop()
{
  stop();
  if (kb_thread_.joinable()) kb_thread_.join();
}

void KeyboardJointTeleop::stop()
{
  running_.store(false);
}

void KeyboardJointTeleop::publishJointCommand()
{
  auto msg = trajectory_msgs::msg::JointTrajectory();
  msg.header.stamp = this->now();
  msg.joint_names = joints_;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  for (const auto &j : joints_) {
    point.positions.push_back(positions_[j]);
  }
  // 더 빠른 반응을 위해 시간을 짧게 설정
  point.time_from_start = rclcpp::Duration::from_seconds(0.05);
  
  msg.points.push_back(point);
  
  // Debug
  RCLCPP_INFO(this->get_logger(), "=== Publishing Joint Trajectory ===");
  RCLCPP_INFO(this->get_logger(), "Joint names (%zu):", msg.joint_names.size());
  for (size_t i = 0; i < msg.joint_names.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "  [%zu] '%s' -> %.3f", 
                i, msg.joint_names[i].c_str(), point.positions[i]);
  }
  RCLCPP_INFO(this->get_logger(), "===================================");
  
  trajectory_pub_->publish(msg);
}

void KeyboardJointTeleop::keyboardLoop()
{
  TermSettings ts; // sets terminal to raw mode while in scope

  int selected_idx = 0;
  if (!joints_.empty()) selected_idx = 0;

  while (rclcpp::ok() && running_.load()) {
    // read a single byte if available
    char c = 0;
    ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n > 0) {
      if (c == 'q' || c == 'Q') {
        RCLCPP_INFO(this->get_logger(), "Quit key pressed");
        running_.store(false);
        break;
      }
      if (c == '+' || c == '=') {
        step_ *= 2.0;
        RCLCPP_INFO(this->get_logger(), "Step -> %f", step_);
      } else if (c == '-') {
        step_ = std::max(1e-6, step_ / 2.0);
        RCLCPP_INFO(this->get_logger(), "Step -> %f", step_);
      } else if (c == 'r' || c == 'R') {
        if (!joints_.empty()) {
          auto j = joints_[selected_idx];
          positions_[j] = 0.0;
          RCLCPP_INFO(this->get_logger(), "[DEBUG] Reset joint '%s' to 0.0", j.c_str());
          publishJointCommand();
        }
      } else if (c >= '1' && c <= '9') {
        int idx = c - '1';
        if (idx < static_cast<int>(joints_.size())) {
          selected_idx = idx;
          RCLCPP_INFO(this->get_logger(), "[DEBUG] Selected joint [%d]: '%s'", 
                      selected_idx, joints_[selected_idx].c_str());
        }
      } else if (c == 'j' || c == 'J') {
        if (!joints_.empty()) {
          auto j = joints_[selected_idx];
          positions_[j] -= step_;
          RCLCPP_INFO(this->get_logger(), "[DEBUG] Decrease joint '%s': %.3f (step=%.3f)", 
                      j.c_str(), positions_[j], step_);
          publishJointCommand();
        }
      } else if (c == 'k' || c == 'K') {
        if (!joints_.empty()) {
          auto j = joints_[selected_idx];
          positions_[j] += step_;
          RCLCPP_INFO(this->get_logger(), "[DEBUG] Increase joint '%s': %.3f (step=%.3f)", 
                      j.c_str(), positions_[j], step_);
          publishJointCommand();
        }
      } else if (c == ',' ) {
        if (!joints_.empty()) {
          selected_idx = (selected_idx - 1 + joints_.size()) % joints_.size();
          RCLCPP_INFO(this->get_logger(), "[DEBUG] Previous joint [%d]: '%s'", 
                      selected_idx, joints_[selected_idx].c_str());
        }
      } else if (c == '.' ) {
        if (!joints_.empty()) {
          selected_idx = (selected_idx + 1) % joints_.size();
          RCLCPP_INFO(this->get_logger(), "[DEBUG] Next joint [%d]: '%s'", 
                      selected_idx, joints_[selected_idx].c_str());
        }
      }
    }

    // sleep briefly and allow rclcpp callbacks
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
