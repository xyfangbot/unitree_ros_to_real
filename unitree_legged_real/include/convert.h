/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _CONVERT_H_
#define _CONVERT_H_

#ifdef USE_ROS1
#include <unitree_legged_msgs/Cartesian.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/IMU.h>
#include <unitree_legged_msgs/LED.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
namespace unitree_msgs = unitree_legged_msgs;
#else
#include <unitree_legged_msgs/msg/cartesian.hpp>
#include <unitree_legged_msgs/msg/high_cmd.hpp>
#include <unitree_legged_msgs/msg/high_state.hpp>
#include <unitree_legged_msgs/msg/imu.hpp>
#include <unitree_legged_msgs/msg/led.hpp>
#include <unitree_legged_msgs/msg/low_cmd.hpp>
#include <unitree_legged_msgs/msg/low_state.hpp>
#include <unitree_legged_msgs/msg/motor_cmd.hpp>
#include <unitree_legged_msgs/msg/motor_state.hpp>
namespace unitree_msgs = unitree_legged_msgs::msg;
#endif

#include "unitree_legged_sdk/unitree_legged_sdk.h"


unitree_msgs::Cartesian state2rosMsg(UNITREE_LEGGED_SDK::Cartesian &state)
{
    unitree_msgs::Cartesian ros_msg;

    ros_msg.x = state.x;
    ros_msg.y = state.y;
    ros_msg.z = state.z;

    return ros_msg;
}

unitree_msgs::IMU state2rosMsg(UNITREE_LEGGED_SDK::IMU &state)
{
    unitree_msgs::IMU ros_msg;

    for (std::size_t i(0); i < 4; ++i)
        ros_msg.quaternion[i] = state.quaternion[i];
    
    for (std::size_t i(0); i < 3; ++i)
    {
        ros_msg.gyroscope[i] = state.gyroscope[i];
        ros_msg.accelerometer[i] = state.accelerometer[i];
        ros_msg.rpy[i] = state.rpy[i];
    }

    ros_msg.temperature = state.temperature;

    return ros_msg;
}

UNITREE_LEGGED_SDK::LED rosMsg2Cmd(const unitree_msgs::LED &msg)
{
    UNITREE_LEGGED_SDK::LED cmd;
    cmd.r = msg.r;
    cmd.g = msg.g;
    cmd.b = msg.b;

    return cmd;
}

UNITREE_LEGGED_SDK::MotorCmd rosMsg2Cmd(const unitree_msgs::MotorCmd &msg)
{
    UNITREE_LEGGED_SDK::MotorCmd cmd;

    cmd.mode = msg.mode;
    cmd.q = msg.q;
    cmd.dq = msg.dq;
    cmd.tau = msg.tau;
    cmd.Kp = msg.kp;
    cmd.Kd = msg.kd;

    for (std::size_t i(0); i < 3; ++i)
        cmd.reserve[i] = msg.reserve[i];
    

    return cmd;
}


unitree_msgs::MotorState state2rosMsg(UNITREE_LEGGED_SDK::MotorState &state)
{
    unitree_msgs::MotorState ros_msg;

    ros_msg.mode = state.mode;
    ros_msg.q = state.q;
    ros_msg.dq = state.dq;
    ros_msg.ddq = state.ddq;
    ros_msg.tau_est = state.tauEst;
    ros_msg.q_raw = state.q_raw;
    ros_msg.dq_raw = state.dq_raw;
    ros_msg.ddq_raw = state.ddq_raw;
    ros_msg.temperature = state.temperature;

    ros_msg.reserve[0] = state.reserve[0];
    ros_msg.reserve[1] = state.reserve[1];

    return ros_msg;
}

UNITREE_LEGGED_SDK::HighCmd rosMsg2Cmd(const unitree_msgs::HighCmd &msg)
{
    UNITREE_LEGGED_SDK::HighCmd cmd;

    cmd.levelFlag = msg.level_flag;
    cmd.commVersion = msg.comm_version;
    cmd.robotID = msg.robot_id;
    cmd.SN = msg.sn;
    cmd.bandWidth = msg.band_width;
    cmd.mode = msg.mode;
    cmd.gaitType = msg.gait_type;
    cmd.speedLevel = msg.speed_level;
    cmd.dFootRaiseHeight = msg.d_foot_raise_height;
    cmd.dBodyHeight = msg.d_body_height;

    for (std::size_t i(0); i < 2; ++i)
    {
        cmd.position[i] = msg.position[i];
        cmd.velocity[i] = msg.velocity[i];
    }

    for (std::size_t i(0); i < 3; ++i)
        cmd.rpy[i] = msg.rpy[i];

    cmd.yawSpeed = msg.yaw_speed;
    
    for (std::size_t i(0); i < 4; ++i)
        cmd.led[i] = rosMsg2Cmd(msg.led[i]);
    

    for (std::size_t i(0); i < 40; ++i)
        cmd.wirelessRemote[i] = msg.wireless_remote[i];
    
    cmd.reserve = msg.reserve;
    cmd.crc = msg.crc;

    return cmd;
}

unitree_msgs::HighState state2rosMsg(UNITREE_LEGGED_SDK::HighState &state)
{
    unitree_msgs::HighState ros_msg;

    ros_msg.level_flag = state.levelFlag;
    ros_msg.comm_version = state.commVersion;
    ros_msg.robot_id = state.robotID;
    ros_msg.sn = state.SN;
    ros_msg.band_width = state.bandWidth;
    ros_msg.mode = state.mode;
    ros_msg.imu = state2rosMsg(state.imu);

    for(std::size_t i(0); i < 3; ++i)
    {
        ros_msg.position[i] = state.position[i];
        ros_msg.velocity[i] = state.velocity[i];
    }

    ros_msg.yaw_speed = state.yawSpeed;

    for(std::size_t i(0); i < 4; ++i)
    {
        ros_msg.foot_position2_body[i] = state2rosMsg(state.footPosition2Body[i]);
        ros_msg.foot_speed2_body[i] = state2rosMsg(state.footSpeed2Body[i]);
        ros_msg.foot_force[i] = state.footForce[i];
    }

    for (std::size_t i(0); i < 40; ++i)
        ros_msg.wireless_remote[i] = state.wirelessRemote[i];
    
    ros_msg.reserve = state.reserve;
    ros_msg.crc = state.crc;
    

    return ros_msg;
}

unitree_msgs::LowState state2rosMsg(UNITREE_LEGGED_SDK::LowState &state)
{
    unitree_msgs::LowState ros_msg;

    ros_msg.level_flag = state.levelFlag;
    ros_msg.comm_version = state.commVersion;
    ros_msg.robot_id = state.robotID;
    ros_msg.sn = state.SN;
    ros_msg.band_width = state.bandWidth;
    ros_msg.imu = state2rosMsg(state.imu);

    for (std::size_t i(0); i < 20; ++i)
        ros_msg.motor_state[i] = state2rosMsg(state.motorState[i]);

    for (std::size_t i(0); i < 4; ++i)
    {
        ros_msg.foot_force[i] = state.footForce[i];
        ros_msg.foot_force_est[i] = state.footForceEst[i];
    }
    
    ros_msg.tick = state.tick;

    for (std::size_t i(0); i < 40; ++i)
        ros_msg.wireless_remote[i] = state.wirelessRemote[i];

    ros_msg.reserve = state.reserve;
    ros_msg.crc = state.crc;


    return ros_msg;
}


UNITREE_LEGGED_SDK::LowCmd rosMsg2Cmd(const unitree_msgs::LowCmd &msg)
{
    UNITREE_LEGGED_SDK::LowCmd cmd;

    cmd.levelFlag = msg.level_flag;
    cmd.commVersion = msg.comm_version;
    cmd.robotID = msg.robot_id;
    cmd.SN = msg.sn;
    cmd.bandWidth = msg.band_width;

    for (std::size_t i(0); i < 20; ++i)
        cmd.motorCmd[i] = rosMsg2Cmd(msg.motor_cmd[i]);
    

    for(std::size_t i(0); i < 4; ++i)
        cmd.led[i] = rosMsg2Cmd(msg.led[i]);

    for (std::size_t i(0); i < 40; ++i)
        cmd.wirelessRemote[i] = msg.wireless_remote[i];

    cmd.reserve = msg.reserve;
    cmd.crc = msg.crc;

    return cmd;
}











#endif // _CONVERT_H_
