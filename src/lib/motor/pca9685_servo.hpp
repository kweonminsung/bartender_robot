#include <PCA9685/PCA9685.hpp>
#include <chrono>
#include <thread>
#include <iostream>
#include <cstdlib>

class PCA9685Servo
{
private:
    PiPCA9685::PCA9685 *pca9685;

public:
    PCA9685Servo();
    ~PCA9685Servo();

    void test(int channel);
    void rotate(int channel, int angle);
    void reset(int channel);
};