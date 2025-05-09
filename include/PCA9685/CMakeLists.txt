# 내부 파일 리스트
set(PCA9685_SOURCES
    src/PCA9685.cpp
    src/I2CPeripheral.cpp
)

# 정적 라이브러리 생성
add_library(PCA9685 STATIC ${PCA9685_SOURCES})

# include path 설정 (내부 및 공개 헤더 포함)
target_include_directories(PCA9685
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# i2c 라이브러리 링크
target_link_libraries(PCA9685 PRIVATE i2c)