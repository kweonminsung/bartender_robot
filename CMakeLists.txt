cmake_minimum_required(VERSION 3.8)
project(bartender_robot)

# --------------------------
# C++ 설정
# --------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------
# ROS 2 패키지 찾기
# --------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

# --------------------------
# 헤더 전용 라이브러리 경로 추가
# --------------------------
include_directories(include)

# --------------------------
# 서드파티 및 내부 라이브러리 빌드
# --------------------------
add_subdirectory(third_party/dynamixel_sdk)

add_library(api_lib src/lib/api/api.cpp)
ament_target_dependencies(api_lib rclcpp)

# --------------------------
# Main Executable
# --------------------------
add_executable(bartender_robot src/main.cpp)

# bartender_robot이 의존하는 ROS 패키지 설정
ament_target_dependencies(bartender_robot
  rclcpp
  sensor_msgs
  urdf
)

# 라이브러리 링크
target_link_libraries(bartender_robot
  dynamixel_sdk
  api_lib
)

# --------------------------
# Visualizer Executable
# --------------------------
add_executable(visualizer src/joint_state_publisher.cpp)

ament_target_dependencies(visualizer
  rclcpp
  sensor_msgs
  urdf
)

target_link_libraries(visualizer
  dynamixel_sdk
)

# --------------------------
# Install rules
# --------------------------
install(TARGETS
  bartender_robot
  visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()