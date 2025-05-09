#include "PCA9685.h"

int main()
{
    // Example usage of PCA9685
    PiPCA9685::PCA9685 pca9685("/dev/i2c-1", 0x40);
    pca9685.set_pwm_freq(60.0); // Set frequency to 60 Hz

    // Set PWM for channel 0
    pca9685.set_pwm(0, 0, 2048); // Set channel 0 to half duty cycle

    return 0;
}