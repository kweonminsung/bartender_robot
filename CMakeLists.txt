cmake_minimum_required(VERSION 3.10)
project(coffee_robot)

set(CMAKE_CXX_STANDARD 17)

# 하위 라이브러리 추가
add_subdirectory(include/PCA9685)

# main.cpp 빌드
add_executable(coffee_robot src/main.cpp)

# 링크: 하위 라이브러리
target_link_libraries(coffee_robot PRIVATE PCA9685)

# include path 설정 (공개 헤더용)
target_include_directories(coffee_robot PRIVATE include)
