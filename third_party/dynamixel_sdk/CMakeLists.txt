add_library(dynamixel_sdk STATIC
    src/dynamixel_sdk/group_bulk_read.cpp
    src/dynamixel_sdk/group_bulk_write.cpp
    src/dynamixel_sdk/group_fast_bulk_read.cpp
    src/dynamixel_sdk/group_fast_sync_read.cpp
    src/dynamixel_sdk/group_handler.cpp
    src/dynamixel_sdk/group_sync_read.cpp
    src/dynamixel_sdk/group_sync_write.cpp
    src/dynamixel_sdk/packet_handler.cpp
    src/dynamixel_sdk/port_handler.cpp
    src/dynamixel_sdk/port_handler_arduino.cpp
    src/dynamixel_sdk/port_handler_linux.cpp
    src/dynamixel_sdk/port_handler_mac.cpp
    src/dynamixel_sdk/port_handler_windows.cpp
    src/dynamixel_sdk/protocol1_packet_handler.cpp
    src/dynamixel_sdk/protocol2_packet_handler.cpp
)

target_include_directories(dynamixel_sdk PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamixel_sdk
)
