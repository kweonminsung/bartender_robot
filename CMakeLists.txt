cmake_minimum_required(VERSION 3.8)
project(bartender_robot)

# Find Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Main Executable
file(GLOB_RECURSE SOURCES
  src/*.cpp
)

add_executable(bartender_robot ${SOURCES})

ament_target_dependencies(bartender_robot
  "rclcpp"
  "std_msgs")

# Link include directories
target_include_directories(bartender_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Install rules
install(TARGETS
  bartender_robot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
