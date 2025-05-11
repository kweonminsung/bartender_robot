#include <chrono>
#include <thread>
#include "PCA9685.h"

int main()
{
    PiPCA9685::PCA9685 pca9685("/dev/i2c-1", 0x42); // 실제 디바이스 주소 확인

    pca9685.set_pwm_freq(60.0); // 서보용 주파수 설정

    while (true)
    {
        // 0번 채널: 시계 방향 (0도 → 180도)
        pca9685.set_pwm(0, 0, 492);

        // 1번 채널: 반시계 방향 (180도 → 0도)
        pca9685.set_pwm(1, 0, 246);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 다시 원위치
        pca9685.set_pwm(0, 0, 246);
        pca9685.set_pwm(1, 0, 492);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

