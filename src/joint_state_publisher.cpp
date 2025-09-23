#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono_literals;

// Control table address
#define ADDR_MX_TORQUE_ENABLE 24
#define ADDR_MX_GOAL_POSITION 30
#define ADDR_MX_PRESENT_POSITION 36

// Protocol version
#define PROTOCOL_VERSION 1.0

// Default setting
#define DXL_ID 1
#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyUSB0"

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

class JointStatePublisher : public rclcpp::Node
{
public:
  JointStatePublisher()
      : Node("joint_state_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = this->create_wall_timer(
        33ms, std::bind(&JointStatePublisher::timer_callback, this));

    portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (portHandler_->openPort())
    {
      RCLCPP_INFO(this->get_logger(), "Succeeded to open the port!");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
      rclcpp::shutdown();
    }

    if (portHandler_->setBaudRate(BAUDRATE))
    {
      RCLCPP_INFO(this->get_logger(), "Succeeded to change the baudrate!");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to change the baudrate!");
      rclcpp::shutdown();
    }

    packetHandler_->write1ByteTxRx(portHandler_, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
  }

  ~JointStatePublisher()
  {
    packetHandler_->write1ByteTxRx(portHandler_, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
    portHandler_->closePort();
  }

private:
  void timer_callback()
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = this->get_clock()->now();
    message.name.push_back("motor_joint");
    message.position.resize(1);

    uint16_t dxl_present_position = 0;
    packetHandler_->read2ByteTxRx(portHandler_, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position);

    message.position[0] = (dxl_present_position - 2048) * 3.14159265359 / 2048.0;
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
