#include "lib/api/api.hpp"
#include "keyboard_teleop.hpp"
#include <rclcpp/rclcpp.hpp>
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

    // Initialize ROS and spin the keyboard teleop node
    rclcpp::init(argc, argv);
    auto keyboard_node = std::make_shared<KeyboardJointTeleop>();

    rclcpp::spin(keyboard_node);

    keyboard_node->stop();
    rclcpp::shutdown();

    return 0;
}