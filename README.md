# Bartender Robot

<img src="public/concepts.png" height="212x">

## Getting started (simulation only)

0. Install System dependencies (see below)

1. Build the workspace

    ```bash
    $ cd bartender_robot
    $ rm -rf log build install # Clean previous builds
    $ colcon build --symlink-install
    ```

2. Source the ROS 2 and workspace overlays

    ```bash
    $ source /opt/ros/rolling/setup.bash
    $ source install/setup.bash
    ```

3. View the robot in RViz

    Starts the `joint_state_publisher` and `robot_state_publisher`, and opens RViz:

    ```bash
    $ ros2 launch bartender_robot rviz.launch.py
    ```

4. Spawn the robot in Gazebo

    ```bash
    $ ros2 launch bartender_robot gz_sim.launch.py
    ```

## System dependencies

Install the packages below(adjust for your ROS 2 distribution) if they are not already available:

```bash
# Install ROS2 before running this
$ sudo apt update
$ sudo apt install \
    ros-rolling-xacro \
    ros-rolling-joint-state-publisher \
    ros-rolling-joint-state-publisher-gui \
    ros-rolling-robot-state-publisher \
    ros-rolling-rviz2

# Install Gazebo and ROS-Gazebo bridge
$ sudo apt install \
    ros-rolling-ros-gz \
    ros-rolling-ros-gz-sim \
    ros-rolling-ros-gz-bridge \
    ros-rolling-ros2-control \
    ros-rolling-ros2-controllers \
    ros-rolling-gz-ros2-control
```


## Full executable including HTTP server

### Roles
- Dynamixel motor control
- HTTP server (provides a simple HTTP API for status/control)

### Getting started

Make sure you have a working CMake toolchain and the required dependencies installed.
From the repository root:

```bash
$ source /opt/ros/rolling/setup.bash
$ mkdir -p build
$ cd build
$ cmake ..
$ make
```

The above will build the native `bartender_robot` executable located in `build/`.

### Run

```bash
# $ source ../env && ros2 run bartender_robot bartender_robot # for environment variables

$ ros2 run bartender_robot bartender_robot --ros-args -p joints:="[revolute_3,revolute_7,revolute_10,revolute_15,revolute_20]"

# or with custom step size
$ ros2 run bartender_robot bartender_robot --ros-args \
  -p joints:="[revolute_3,revolute_7,revolute_10,revolute_15,revolute_20]" \
  -p step:=0.5
```

### Dependencies
- [CMake](https://cmake.org/)
- [nlohmann_json](https://github.com/nlohmann/json)
- [cpp-httplib](https://github.com/yhirose/cpp-httplib)
