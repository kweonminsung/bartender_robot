#include "api.hpp"
#include "keyboard_teleop.hpp"
#include "dynamixel_controller.hpp"
#include "plan_player.hpp"
#include "joint_state_publisher.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <memory>
#include <thread>

int main(int argc, char **argv)
{
    // Initialize ROS first
    rclcpp::init(argc, argv);
    
    // Create plan player node
    auto plan_player_node = std::make_shared<PlanPlayer>();
    
    // Create API server and set callback before starting to listen
    auto api_server = std::make_shared<ApiServer>();
    api_server->set_play_plan_callback([plan_player_node](const std::string& csv_path) {
        plan_player_node->loadAndPublishPlan(csv_path);
    });
    
    // Start API server in a background thread because start_listening() blocks
    std::thread api_thread([api_server]() {
        api_server->start_listening();
    });
    api_thread.detach();
    
    // Create a temporary node to read parameters
    auto temp_node = std::make_shared<rclcpp::Node>("temp_param_node");
    temp_node->declare_parameter<bool>("use_dynamixel", false);
    bool use_dynamixel = temp_node->get_parameter("use_dynamixel").as_bool();
    temp_node->declare_parameter<bool>("use_motion_planning", false);
    bool use_motion_planning = temp_node->get_parameter("use_motion_planning").as_bool();
    temp_node->declare_parameter<bool>("publish_joint_states", true);
    bool publish_joint_states = temp_node->get_parameter("publish_joint_states").as_bool();
    
    auto keyboard_node = std::make_shared<KeyboardJointTeleop>();
    auto joint_state_pub_node = std::make_shared<JointStatePublisher>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(keyboard_node);
    executor.add_node(plan_player_node);
    executor.add_node(joint_state_pub_node);

    // Conditionally create and add dynamixel node
    std::shared_ptr<DynamixelController> dynamixel_node;
    if (use_dynamixel) {
        dynamixel_node = std::make_shared<DynamixelController>();
        executor.add_node(dynamixel_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "Dynamixel controller enabled");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Dynamixel controller disabled");
    }
    
    executor.spin();

    keyboard_node->stop();

    rclcpp::shutdown();

    return 0;
}