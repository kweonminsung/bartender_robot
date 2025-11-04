cmake_minimum_required(VERSION 3.8)
project(bartender_robot)

# Find Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Note: this package contains URDF/xacro and meshes for visualization.
# Install the urdf, meshes and launch files into share/<pkg>

# Main Executable
file(GLOB_RECURSE SOURCES
  src/*.cpp
)

add_executable(bartender_robot ${SOURCES})

target_link_libraries(bartender_robot PUBLIC
      ${std_msgs_TARGETS}
      ${trajectory_msgs_TARGETS}
      rclcpp::rclcpp
    )

# Link include directories
target_include_directories(bartender_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

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
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
