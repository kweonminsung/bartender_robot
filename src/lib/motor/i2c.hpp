#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <vector>
#include <cstdlib>

#define PCA9685_CHANNEL_BASE 0x06
#define CHANNEL_COUNT 16

// inline std::vector<int> scan_i2c_bus(const char *i2c_device)
// {
//     int file;
//     std::vector<int> found_bus;

//     if ((file = open(i2c_device, O_RDWR)) < 0)
//     {
//         std::cerr << "Failed to open I2C bus: " << i2c_device << std::endl;
//         return found_bus;
//     }

//     for (int addr = 0x03; addr <= 0x77; ++addr)
//     {
//         if (ioctl(file, I2C_SLAVE, addr) < 0)
//         {
//             continue; // Unable to set device address
//         }

//         // Try a dummy write (zero-length) to check if device responds
//         unsigned char buf[1] = {0};
//         if (write(file, buf, 1) == 1)
//         {
//             found_bus.push_back(addr);
//         }
//     }

//     close(file);
//     return found_bus;
// }

inline std::vector<int> scan_i2c_bus_channel(const char *i2c_device, int i2c_address)
{
    int file;
    std::vector<int> found_channels;

    if ((file = open(i2c_device, O_RDWR)) < 0)
    {
        std::cerr << "Failed to open I2C bus: " << i2c_device << std::endl;
        exit(EXIT_FAILURE);
    }

    if (ioctl(file, I2C_SLAVE, i2c_address) < 0)
    {
        std::cerr << "Failed to set I2C address: " << i2c_address << std::endl;
        close(file);
        exit(EXIT_FAILURE);
    }

    for (int channel = 0; channel < CHANNEL_COUNT; ++channel)
    {
        uint8_t reg = PCA9685_CHANNEL_BASE + 4 * channel;
        uint8_t data[4] = {0};

        if (write(file, &reg, 1) != 1)
        {
            // std::cerr << "Failed to select channel register for channel " << channel << "\n";
            continue;
        }

        if (read(file, data, 4) != 4)
        {
            // std::cerr << "Failed to read channel " << channel << " data\n";
            continue;
        }

        int on = data[0] | (data[1] << 8);
        int off = data[2] | (data[3] << 8);

        if (on != 0 || off != 0)
        {
            found_channels.push_back(channel);
        }
    }
    close(file);

    return found_channels;
}