Packages Version: v3.8.4

# Introduction
This package can send control command to real robot from ROS. You can do low-level control(namely control all joints on robot) and high-level control(namely control the walking direction and speed of robot).

This version is suitable for unitree_legged_sdk v3.8.4, namely Aliengo robot.

## Packages:

Basic message function: `unitree_legged_msgs`

The interface between ROS and real robot: `unitree_legged_real`

## Environment
We recommand users to run this package in Ubuntu 20.04 and ROS neotic environment

## Dependency
* [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk/releases)
### Notice
The release [v3.8.4](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.8.4) only supports for robot: Aliengo.

# Build
You can use catkin_make to build ROS packages. First copy the package folder to `~/catkin_ws/src`, then download the corresponding unitree_legged_sdk into `~/catkin_ws/src/unitree_ros_to_real`.Be careful with the sdk folder name. It should be "unitree_legged_sdk" without version tag:
```
cd ~/catkin_ws
catkin_make
```

# Run the package
You can control your real Aliengo robot from ROS by this package.

Before you run expamle program, please run 

```
roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
```
or
```
roslaunch unitree_legged_real real.launch ctrl_level:=lowlevel
```

It depends which control mode you want to use.

Then, if you want to run high-level control mode, you can run example_walk node 
```
rosrun unitree_legged_real example_walk
```

If you want to run low-level control mode, you can run example_position program node 
```
rosrun unitree_legged_real example_position
```

And before you do the low-level control, please press L2+A to sit the robot down and then press L1+L2+start to make the robot into
mode in which you can do low-level control, finally make sure you hang the robot up before you run low-level control.

## ROS1 / ROS2 compatibility

Source the desired ROS environment before configuring. As in fastlio2,
`ROS_VERSION` selects catkin / `USE_ROS1` or ament / `USE_ROS2`.
All nine message types use a single shared `.msg` definition. Message type names
(including `IMU` and `LED`) are unchanged; fields now use lowercase snake_case
in both ROS versions, for example `level_flag`, `motor_cmd`, `motor_state`,
`yaw_speed`, `kp`, and `kd`. Existing ROS1 consumers must update their field
accesses. The converters retain the SDK's original member names.

Add SDK v3.8.4 in `unitree_ros_to_real/unitree_legged_sdk` before building.
Alternatively, set the CMake cache variable `UNITREE_SDK_DIR` to its location.
The library is selected from `lib/cpp/amd64` or `lib/cpp/arm64`.

For ROS2, build later with colcon and source the resulting install environment.
Run the bridge with:

```sh
ros2 launch unitree_legged_real real.launch.xml ctrl_level:=highlevel
# Or use ctrl_level:=lowlevel
ros2 run unitree_legged_real example_walk
# For low-level control, use example_position instead.
```

The existing ROS1 `real.launch` and `rosrun` commands remain available.
