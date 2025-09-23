cmake_minimum_required(VERSION 3.10)
project(bartender_robot)

set(CMAKE_CXX_STANDARD 17)

# Dynamixel SDK
add_subdirectory(third_party/dynamixel_sdk)

# OpenSSL 필요 (cpp-httplib SSL 지원)
# find_package(OpenSSL REQUIRED)

add_library(nlohmann_json
    INTERFACE
)
add_library(httplib
    INTERFACE
)
# target_compile_definitions(httplib INTERFACE CPPHTTPLIB_OPENSSL_SUPPORT)
# target_link_libraries(httplib INTERFACE OpenSSL::SSL OpenSSL::Crypto)

add_executable(bartender_robot
    src/main.cpp
    src/lib/api/api.cpp
    # src/lib/llm/llm.cpp
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
        httplib
        dynamixel_sdk
)
