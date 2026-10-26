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

## AlienGo planner controller

`aliengo_ctrl_node` directly owns the SDK high-level UDP connection. Run it
instead of `ros_udp` or the SDK control examples (the default local port is 8081).
It subscribes to `geometry_msgs/TwistStamped` on `/cmd_planner`; `linear.x` is
forward speed in m/s and `angular.z` is yaw rate in rad/s, both in the robot body
frame. Other twist components are ignored. The publisher must stamp messages
with the same ROS clock as the controller; `frame_id` does not perform a transform.

```sh
# ROS1
roslaunch unitree_legged_real aliengo_ctrl.launch
# ROS2
ros2 launch unitree_legged_real aliengo_ctrl.launch.xml
```

Parameters: `cmd_topic` (default `/cmd_planner`), `target_ip`
(`192.168.123.220`), `local_port` (8081), and `target_port` (8082).
The launch files expose `cmd_topic` and `target_ip`; ports can be set as node
parameters when running the executable directly.

At startup the node sends the sport trigger from
`example_start_aliengo_sport.cpp` (`levelFlag=0xf0` for 90 ms), restores
`HIGHLEVEL`, waits 2 seconds, then commands modes 7 (0.5 s), 5 (1.8 s),
6 (3.5 s), and 1 (1.5 s), before entering velocity mode 2.
Velocities remain zero throughout startup; a fresh planner message must arrive
after startup completes before movement is allowed.

The transition order follows Unitree's
[AlienGo mode guide](https://github.com/UnitreeSupport/Unitree_Docs/blob/master/docs/Aliengo/AlienGo.md)
for motion programs 1.0.17 and later: stand `7 -> 5 -> 6`, enable walking
`6 -> 1 -> 2`, crouch `2 -> 1 -> 6 -> 5 -> 7`. This corrects the differing
mode order/comments in `example_end_aliengo_sport.cpp`. Dwell times are based on
the local SDK example and are estimates; the node does not confirm posture
completion from feedback. Validate them against the installed motion program.

Commands are sent at 500 Hz. A message older than 100 ms, a gap of more than
100 ms since receipt of a new valid message, a future/zero/replayed timestamp,
or non-finite velocity clears the velocity command. Both ROS timestamp age
and steady elapsed time are checked on each send. Timeout keeps mode 2 with
zero velocity; it does not start the sit sequence. ROS2 uses volatile,
best-effort QoS with depth 1 (compatible with reliable or best-effort publishers).

On SIGINT/SIGTERM or ROS shutdown, the node sends zero velocity in mode 2
for 0.2 s, then modes 1 (1.5 s), 6 (1 s), 5 (2 s), and 7 (1 s), before
closing UDP. Startup interruption skips unnecessary stand-up stages. Allow at
least 6 seconds for graceful exit. SIGKILL, process crashes, network loss, or
power loss cannot guarantee that the robot receives this sequence.

### Keyboard planner commands

Run in an interactive terminal after sourcing ROS (no SDK build is needed to
run the script directly):

```sh
python3 src/unitree_ros_to_real/unitree_legged_real/scripts/aliengo_keyboard.py
```

After installing the package, use `rosrun unitree_legged_real aliengo_keyboard.py`
or `ros2 run unitree_legged_real aliengo_keyboard.py`.
W/S increment/decrement forward speed by 0.05 m/s. A/D increment/decrement
yaw rate by 0.05 rad/s (positive turns left). Commands start at `(0, 0)` and
persist while the script publishes fresh TwistStamped messages at 50 Hz.
Space resets both values; Q or Ctrl-C sends zero and exits. Key auto-repeat
also increments the command. These are yaw turns, rather than lateral motion,
to match `aliengo_ctrl_node`'s `linear.x` / `angular.z` interface.
