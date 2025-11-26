#include "api.hpp"
#include "keyboard_teleop.hpp"
#include "dynamixel_controller.hpp"
#include "motion_planning_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <memory>
#include <thread>

int main(int argc, char **argv)
{
    // Start API server in a background thread because ApiServer::ApiServer() blocks on listen().
    std::thread api_thread([]() {
        ApiServer apiServer;
        // ApiServer will block until it stops. When the process exits the thread will end.
    });
    api_thread.detach();

    // Initialize ROS
    rclcpp::init(argc, argv);
    
    // Create a temporary node to read parameters
    auto temp_node = std::make_shared<rclcpp::Node>("temp_param_node");
    temp_node->declare_parameter<bool>("use_dynamixel", false);
    bool use_dynamixel = temp_node->get_parameter("use_dynamixel").as_bool();
    temp_node->declare_parameter<bool>("use_motion_planning", false);
    bool use_motion_planning = temp_node->get_parameter("use_motion_planning").as_bool();
    
    auto keyboard_node = std::make_shared<KeyboardJointTeleop>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(keyboard_node);

    // Conditionally create and add motion planning node
    if (use_motion_planning) {
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        auto motion_planning_node = std::make_shared<MotionPlanningNode>(node_options);
        executor.add_node(motion_planning_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "Motion planning enabled");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Motion planning disabled");
    }
    
    // Conditionally create and add dynamixel node
    if (use_dynamixel) {
        auto dynamixel_node = std::make_shared<DynamixelController>();
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