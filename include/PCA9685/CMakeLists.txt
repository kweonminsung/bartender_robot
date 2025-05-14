# 정적 라이브러리 생성
add_library(PCA9685
    STATIC
        src/PCA9685.cpp
        src/I2CPeripheral.cpp
)

# include path 설정
target_include_directories(PCA9685
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 라이브러리 링크
target_link_libraries(PCA9685 PRIVATE i2c)