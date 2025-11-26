#include "joint_state_publisher.hpp"

JointStatePublisher::JointStatePublisher() : Node("joint_state_publisher")
{
    // Declare parameters
    this->declare_parameter<bool>("publish_joint_states", true);
    this->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{
        "XL_430_base_Revolute-29",
        "XL_430_shoulder_Revolute-30",
        "XL_430_upper_Revolute-31",
        "XL_430_lower_Revolute-32"
    });

    publish_joint_states_ = this->get_parameter("publish_joint_states").as_bool();
    joint_names_ = this->get_parameter("joints").as_string_array();

    if (publish_joint_states_) {
        // Create publisher for joint states
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        
        // Subscribe to joint positions from other nodes
        joint_position_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/joint_positions", 10,
            std::bind(&JointStatePublisher::jointPositionCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Joint State Publisher enabled - publishing to /joint_states");
    } else {
        RCLCPP_INFO(this->get_logger(), "Joint State Publisher disabled");
    }
    
    RCLCPP_INFO(this->get_logger(), "Configured %zu joints", joint_names_.size());
}

void JointStatePublisher::jointPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!publish_joint_states_) {
        return;
    }

    if (msg->data.size() != joint_names_.size()) {
        RCLCPP_WARN(this->get_logger(), 
            "Received %zu positions but expected %zu joints", 
            msg->data.size(), joint_names_.size());
        return;
    }

    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name = joint_names_;
    joint_state_msg.position = msg->data;

    joint_state_pub_->publish(joint_state_msg);
}
