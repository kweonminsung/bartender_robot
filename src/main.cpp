#include "api.hpp"
#include "keyboard_teleop.hpp"
#include "dynamixel_controller.hpp"
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
    
    auto keyboard_node = std::make_shared<KeyboardJointTeleop>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(keyboard_node);
    
    // Conditionally create and add dynamixel node
    std::shared_ptr<DynamixelController> dynamixel_node = nullptr;
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