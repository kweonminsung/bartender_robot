#ifndef JOINT_STATE_PUBLISHER_HPP
#define JOINT_STATE_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <vector>

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher();

private:
    void jointPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_sub_;
    
    std::vector<std::string> joint_names_;
    bool publish_joint_states_;
};

#endif // JOINT_STATE_PUBLISHER_HPP
