#include "pca9685_servo.hpp"
#include "i2c.hpp"

PCA9685Servo::PCA9685Servo()
{
    const char *i2c_device = std::getenv("I2C_DEVICE");
    int i2c_address = std::getenv("I2C_ADDRESS") ? std::atoi(std::getenv("I2C_ADDRESS")) : 0x42;

    if (i2c_device == nullptr)
    {
        std::cerr << "I2C_DEVICE environment variable not set." << std::endl;
        exit(EXIT_FAILURE);
    }

    // I2C 장치 스캔
    for (int channel : scan_i2c_bus_channel(i2c_device, i2c_address))
    {
        std::cout << "Found PCA9685 channel: " << channel << std::endl;
    }

    this->pca9685 = new PiPCA9685::PCA9685(i2c_device, i2c_address);

    this->pca9685->set_pwm_freq(60.0); // 서보용 주파수 설정
}

PCA9685Servo::~PCA9685Servo()
{
    delete this->pca9685;
}

void PCA9685Servo::test(int channel)
{
    // 테스트용으로 0도에서 180도까지 회전
    for (int angle = 0; angle <= 180; angle += 10)
    {
        this->rotate(channel, angle);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 다시 0도로 돌아오기
    for (int angle = 180; angle >= 0; angle -= 10)
    {
        this->rotate(channel, angle);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    this->reset(channel);
}

void PCA9685Servo::rotate(int channel, int angle)
{
    // 각도에 따라 PWM 값 계산
    uint16_t pwm_value = static_cast<uint16_t>(angle * 4096 / 180);
    this->pca9685->set_pwm(channel, 0, pwm_value);
}

void PCA9685Servo::reset(int channel)
{
    this->pca9685->set_pwm(channel, 0, 0); // PWM 값 초기화
}
