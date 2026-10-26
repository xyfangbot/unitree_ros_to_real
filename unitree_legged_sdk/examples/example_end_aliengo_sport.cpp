/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace UNITREE_LEGGED_SDK;

// high cmd
constexpr uint16_t TARGET_PORT = 8082;
constexpr uint16_t LOCAL_PORT = 8081;
constexpr char TARGET_IP[] = "192.168.123.220";   // target IP address


//// low cmd
//constexpr uint16_t TARGET_PORT = 8007;
//constexpr uint16_t LOCAL_PORT = 8082;
//constexpr char TARGET_IP[] = "192.168.123.10";   // target IP address

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::Aliengo),
        udp(LOCAL_PORT, TARGET_IP,TARGET_PORT, sizeof(HighCmd), sizeof(HighState))
    {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);

    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.position[0] = 0.0f;
    cmd.position[1] = 0.0f;
    cmd.yawSpeed = 0.0f;;

    cmd.mode = 0;
    cmd.rpy[0]  = 0;
    cmd.rpy[1] = 0;
    cmd.rpy[2] = 0;
    cmd.gaitType = 0;
    cmd.dBodyHeight = 0;
    cmd.dFootRaiseHeight = 0;

    if (motiontime == 2)
    {
        std::cout<<"begin sending commands."<<std::endl;
    }


    if (motiontime>200 && motiontime <2000)
    {
        cmd.mode = 5; // force stand
    }
    if (motiontime>2000 && motiontime <3500)
    {
        cmd.mode = 1; // force stand
    }
    if (motiontime>3500 && motiontime <7000)
    {
        cmd.mode = 6; // stand up
    }
    if (motiontime>7000 && motiontime <9000)
    {
        cmd.mode = 5; // stand down
    }
    if (motiontime>9000 && motiontime <10000)
    {
        cmd.mode = 7; // damping mode
    }
    // if (motiontime>8000 && motiontime <8500)
    // {
    //     cmd.levelFlag = 0x00;
    // }

    if (motiontime == 8000)
        std::cout<<"Aliengo sport mode finished !"<<std::endl;

    udp.SetSend(cmd);
}

int main(void) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
