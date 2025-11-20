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

    // Initialize ROS and spin each node
    rclcpp::init(argc, argv);
    
    auto keyboard_node = std::make_shared<KeyboardJointTeleop>();
    auto dynamixel_node = std::make_shared<DynamixelController>();

    // Use MultiThreadedExecutor to spin both nodes
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(keyboard_node);
    executor.add_node(dynamixel_node);
    
    executor.spin();

    keyboard_node->stop();

    rclcpp::shutdown();

    return 0;
}