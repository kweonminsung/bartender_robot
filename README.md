# Bartender Robot

<!-- <img src="public/concepts.png" width="420px" height="212x"> -->

### Roles
- Dynamixel Motor Control
- HTTP Server

### How to Build
```bash
$ source /opt/ros/rolling/setup.bash # adjust according to your ROS 2 version

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
