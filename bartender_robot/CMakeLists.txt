cmake_minimum_required(VERSION 3.8)
project(bartender_robot)

# Find Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# Main Executable
add_executable(bartender_robot 
  src/main.cpp
  src/keyboard_teleop.cpp
  src/dynamixel_controller.cpp
  src/api.cpp
  src/plan_player.cpp
  src/joint_state_publisher.cpp
)

target_compile_features(bartender_robot PUBLIC cxx_std_17)

target_link_libraries(bartender_robot PUBLIC
  ${std_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  dynamixel_sdk::dynamixel_sdk
  rclcpp::rclcpp
)

# Link include directories
target_include_directories(bartender_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install rules
install(TARGETS
  bartender_robot
  DESTINATION lib/${PROJECT_NAME}
)

# Install resources so launch files can find them via ament index
install(DIRECTORY
  urdf
  meshes
  launch
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()