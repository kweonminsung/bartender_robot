# Bartender Robot

### Roles
- I2C Controller
- HTTP Server

### How to Build
```bash
$ mkdir build # if build directory not exists

$ cd build
$ cmake ..
$ make
```

### Run
```bash
$ source ../env && ./bartender_robot
```

### Dependencies
- [CMake](https://cmake.org/)
- [nlohmann_json](https://github.com/nlohmann/json)
- [cpp-httplib](https://github.com/yhirose/cpp-httplib)


### Etc
- **fatal error: i2c/smbus.h: No such file or directory 에러 발생 시

    ```bash
    $ apt install libi2c-dev
    ```
