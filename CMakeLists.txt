cmake_minimum_required(VERSION 3.10)
project(bartender_robot)

set(CMAKE_CXX_STANDARD 17)

# 하위 라이브러리 추가(빌드 포함)
# add_subdirectory(
#     include/PCA9685
# )

add_library(nlohmann_json
    INTERFACE
)

add_executable(bartender_robot
    src/main.cpp
    src/lib/api/api.cpp
    # src/lib/motor/pca9685_servo.cpp
)

# include path 설정
target_include_directories(bartender_robot
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include

)

# 라이브러리 링크
target_link_libraries(bartender_robot
    PRIVATE
        nlohmann_json
        # PCA9685
)

