#include "dynamixel_controller.hpp"
#include <cmath>

DynamixelController::DynamixelController()
: Node("dynamixel_controller")
{
  // Declare parameters
  this->declare_parameter<std::string>("device_name", DEVICENAME);
  this->declare_parameter<int>("baudrate", BAUDRATE);
  this->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
  this->declare_parameter<std::vector<int64_t>>("dynamixel_ids", std::vector<int64_t>{});

  // Get parameters
  device_name_ = this->get_parameter("device_name").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  joint_names_ = this->get_parameter("joints").as_string_array();
  auto dynamixel_ids = this->get_parameter("dynamixel_ids").as_integer_array();

  if (joint_names_.empty() || dynamixel_ids.empty()) {
    RCLCPP_ERROR(this->get_logger(), "joints or dynamixel_ids not provided!");
    return;
  }

  if (joint_names_.size() != dynamixel_ids.size()) {
    RCLCPP_ERROR(this->get_logger(), "joints and dynamixel_ids size mismatch!");
    return;
  }

  // Map joint names to Dynamixel IDs
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    joint_id_map_[joint_names_[i]] = static_cast<uint8_t>(dynamixel_ids[i]);
    joint_positions_[joint_names_[i]] = 0.0;
    RCLCPP_INFO(this->get_logger(), "Joint '%s' mapped to Dynamixel ID %ld", 
                joint_names_[i].c_str(), dynamixel_ids[i]);
  }

  // Setup Dynamixel
  if (!setupDynamixel()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup Dynamixel!");
    return;
  }

  // Create subscriber for trajectory commands
  trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_controller/joint_trajectory",
    10,
    std::bind(&DynamixelController::trajectoryCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "DynamixelController initialized successfully");
}

DynamixelController::~DynamixelController()
{
  shutdownDynamixel();
}

bool DynamixelController::setupDynamixel()
{
  // Initialize PortHandler
  port_handler_ = std::unique_ptr<dynamixel::PortHandler>(
    dynamixel::PortHandler::getPortHandler(device_name_.c_str())
  );

  // Initialize PacketHandler
  packet_handler_ = std::unique_ptr<dynamixel::PacketHandler>(
    dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION)
  );

  // Open port
  if (!port_handler_->openPort()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open port %s", device_name_.c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Succeeded to open port %s", device_name_.c_str());

  // Set baudrate
  if (!port_handler_->setBaudRate(baudrate_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate %d", baudrate_);
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Succeeded to set baudrate %d", baudrate_);

  // Enable torque for all motors
  for (const auto& pair : joint_id_map_) {
    uint8_t id = pair.second;
    
    // Disable torque first to change settings
    if (!setTorqueEnable(id, false)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to disable torque for ID %d", id);
      return false;
    }

    // Set operating mode to Extended Position Control Mode (4)
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_handler_->write1ByteTxRx(
      port_handler_.get(), id, ADDR_OPERATING_MODE, 4, &dxl_error
    );
    
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set operating mode for ID %d: %s", 
                   id, packet_handler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    // Set Profile Velocity to 25
    dxl_comm_result = packet_handler_->write4ByteTxRx(
      port_handler_.get(), id, ADDR_PROFILE_VELOCITY, 25, &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set profile velocity for ID %d: %s", 
                   id, packet_handler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    // Set Position D Gain to 0
    dxl_comm_result = packet_handler_->write2ByteTxRx(
      port_handler_.get(), id, ADDR_POSITION_D_GAIN, 0, &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set position d gain for ID %d: %s", 
                   id, packet_handler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    // Set Position P Gain to 600
    dxl_comm_result = packet_handler_->write2ByteTxRx(
      port_handler_.get(), id, ADDR_POSITION_P_GAIN, 600, &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set position p gain for ID %d: %s", 
                   id, packet_handler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    // Enable torque
    if (!setTorqueEnable(id, true)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for ID %d", id);
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Enabled torque for Dynamixel ID %d", id);
  }

  return true;
}

void DynamixelController::shutdownDynamixel()
{
  if (!port_handler_ || !packet_handler_) return;

  // Disable torque for all motors
  for (const auto& pair : joint_id_map_) {
    setTorqueEnable(pair.second, false);
  }

  // Close port
  if (port_handler_) {
    port_handler_->closePort();
  }
  
  RCLCPP_INFO(this->get_logger(), "Dynamixel shutdown complete");
}

bool DynamixelController::setTorqueEnable(uint8_t id, bool enable)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = packet_handler_->write1ByteTxRx(
    port_handler_.get(), id, ADDR_TORQUE_ENABLE, enable ? 1 : 0, &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set torque for ID %d: %s", 
                 id, packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  if (dxl_error != 0) {
    RCLCPP_ERROR(this->get_logger(), "Dynamixel error for ID %d: %s", 
                 id, packet_handler_->getRxPacketError(dxl_error));
    return false;
  }

  return true;
}

bool DynamixelController::setGoalPosition(uint8_t id, int32_t position)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = packet_handler_->write4ByteTxRx(
    port_handler_.get(), id, ADDR_GOAL_POSITION, position, &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set goal position for ID %d: %s", 
                 id, packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  if (dxl_error != 0) {
    RCLCPP_ERROR(this->get_logger(), "Dynamixel error for ID %d: %s", 
                 id, packet_handler_->getRxPacketError(dxl_error));
    return false;
  }

  return true;
}

int32_t DynamixelController::getPresentPosition(uint8_t id)
{
  uint8_t dxl_error = 0;
  int32_t present_position = 0;
  int dxl_comm_result = packet_handler_->read4ByteTxRx(
    port_handler_.get(), id, ADDR_PRESENT_POSITION, 
    reinterpret_cast<uint32_t*>(&present_position), &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "Failed to read present position for ID %d: %s", 
                id, packet_handler_->getTxRxResult(dxl_comm_result));
    return 0;
  }

  if (dxl_error != 0) {
    RCLCPP_WARN(this->get_logger(), "Dynamixel error for ID %d: %s", 
                id, packet_handler_->getRxPacketError(dxl_error));
    return 0;
  }

  return present_position;
}

int32_t DynamixelController::radianToPosition(double radian)
{
  // XL430-W250: 4096 positions per revolution (0 ~ 4095)
  // 1 revolution = 2*PI radians
  // Center position = 2048 (corresponds to 0 radians)
  
  double position = (radian / (2.0 * M_PI)) * 4096.0 + 2048.0;
  
  // Clamp to valid range
  if (position < 0) position = 0;
  if (position > 4095) position = 4095;
  
  return static_cast<int32_t>(position);
}

double DynamixelController::positionToRadian(int32_t position)
{
  // Convert position to radian
  // Center position = 2048 (corresponds to 0 radians)
  
  return ((position - 2048.0) / 4096.0) * 2.0 * M_PI;
}

void DynamixelController::trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
    return;
  }

  // Use the first point
  const auto& point = msg->points[0];
  
  if (msg->joint_names.size() != point.positions.size()) {
    RCLCPP_ERROR(this->get_logger(), "Joint names and positions size mismatch");
    return;
  }

  // Set goal position for each joint
  for (size_t i = 0; i < msg->joint_names.size(); ++i) {
    const std::string& joint_name = msg->joint_names[i];
    double position_rad = point.positions[i];
    
    auto it = joint_id_map_.find(joint_name);
    if (it == joint_id_map_.end()) {
      RCLCPP_WARN(this->get_logger(), "Unknown joint: %s", joint_name.c_str());
      continue;
    }
    
    uint8_t id = it->second;
    int32_t position_value = radianToPosition(position_rad);
    
    if (setGoalPosition(id, position_value)) {
      joint_positions_[joint_name] = position_rad;
      RCLCPP_DEBUG(this->get_logger(), "Set joint '%s' (ID %d) to %.3f rad (%d)", 
                   joint_name.c_str(), id, position_rad, position_value);
    }
  }
}
