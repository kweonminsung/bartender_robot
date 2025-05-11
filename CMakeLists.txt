cmake_minimum_required(VERSION 3.10)
project(bartender_robot)

set(CMAKE_CXX_STANDARD 17)

# 하위 라이브러리 추가
# add_subdirectory(include/PCA9685)

# main.cpp 빌드
add_executable(bartender_robot
    src/main.cpp
    src/lib/api/api.cpp
)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/lib/api
)

# 링크: 하위 라이브러리
# target_link_libraries(bartender_robot PRIVATE PCA9685)

# include path 설정 (공개 헤더용)
target_include_directories(bartender_robot PRIVATE include)
