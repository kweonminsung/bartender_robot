cmake_minimum_required(VERSION 3.8)
project(bartender_robot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

# Dynamixel SDK
add_subdirectory(third_party/dynamixel_sdk)

# Original executable
add_library(nlohmann_json INTERFACE)
add_library(httplib INTERFACE)

add_executable(bartender_robot
    src/main.cpp
    src/lib/api/api.cpp
)
target_include_directories(bartender_robot
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(bartender_robot
    PRIVATE
        nlohmann_json
        httplib
        dynamixel_sdk
)

# Visualizer executable
add_executable(visualizer src/joint_state_publisher.cpp)
ament_target_dependencies(visualizer rclcpp sensor_msgs urdf dynamixel_sdk)

install(TARGETS
  bartender_robot
  visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
