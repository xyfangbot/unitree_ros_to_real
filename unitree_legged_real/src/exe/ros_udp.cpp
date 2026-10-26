#include <iostream>
#ifdef USE_ROS1
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <boost/bind/bind.hpp>
#include <strings.h>

using namespace UNITREE_LEGGED_SDK;

const int LOW_CMD_LENGTH = 610;
const int LOW_STATE_LENGTH = 771;

class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : 
        // low_udp(LOWLEVEL),
        low_udp(8082, "192.168.123.10", 8007, LOW_CMD_LENGTH, LOW_STATE_LENGTH),
        high_udp(8081, "192.168.123.220", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {

        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

Custom custom;

#ifdef USE_ROS1
ros::Subscriber sub_high;
ros::Subscriber sub_low;
ros::Publisher pub_high;
ros::Publisher pub_low;
#else
rclcpp::Subscription<unitree_msgs::HighCmd>::SharedPtr sub_high;
rclcpp::Subscription<unitree_msgs::LowCmd>::SharedPtr sub_low;
rclcpp::Publisher<unitree_msgs::HighState>::SharedPtr pub_high;
rclcpp::Publisher<unitree_msgs::LowState>::SharedPtr pub_low;
#endif

long high_count = 0;
long low_count = 0;

#ifdef USE_ROS1
void highCmdCallback(const unitree_msgs::HighCmd::ConstPtr &msg)
#else
void highCmdCallback(const unitree_msgs::HighCmd::ConstSharedPtr &msg)
#endif
{
    printf("highCmdCallback is running !\t%ld\n", ::high_count++);

    custom.high_cmd = rosMsg2Cmd(*msg);
    
    unitree_msgs::HighState high_state_ros;

    high_state_ros = state2rosMsg(custom.high_state);

#ifdef USE_ROS1
    pub_high.publish(high_state_ros);
#else
    pub_high->publish(high_state_ros);
#endif

    // printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);
}

#ifdef USE_ROS1
void lowCmdCallback(const unitree_msgs::LowCmd::ConstPtr &msg)
#else
void lowCmdCallback(const unitree_msgs::LowCmd::ConstSharedPtr &msg)
#endif
{

    printf("lowCmdCallback is running !\t%ld\n", ::low_count++);

    custom.low_cmd = rosMsg2Cmd(*msg);

    unitree_msgs::LowState low_state_ros;

    low_state_ros = state2rosMsg(custom.low_state);

#ifdef USE_ROS1
    pub_low.publish(low_state_ros);
#else
    pub_low->publish(low_state_ros);
#endif

    // printf("lowCmdCallback ending!\t%ld\n\n", ::low_count++);
}

int main(int argc, char **argv)
{
#ifdef USE_ROS1
    ros::init(argc, argv, "ros_udp");
#else
    rclcpp::init(argc, argv);
#endif

#ifdef USE_ROS1
    ros::NodeHandle nh;
#else
    auto nh = rclcpp::Node::make_shared("ros_udp");
#endif

    // Ignore ROS remapping arguments when selecting the positional control level.
#ifdef USE_ROS1
    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);
#else
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
#endif
    if (args.size() != 2)
    {
        std::cerr << "Usage: ros_udp HIGHLEVEL|LOWLEVEL" << std::endl;
        return 1;
    }

    if (strcasecmp(args[1].c_str(), "LOWLEVEL") == 0)
    {
#ifdef USE_ROS1
        sub_low = nh.subscribe("low_cmd", 1, lowCmdCallback);
        pub_low = nh.advertise<unitree_msgs::LowState>("low_state", 1);
#else
        sub_low = nh->create_subscription<unitree_msgs::LowCmd>("low_cmd", 1, lowCmdCallback);
        pub_low = nh->create_publisher<unitree_msgs::LowState>("low_state", 1);
#endif

        LoopFunc loop_udpSend("low_udp_send", 0.002, 3, boost::bind(&Custom::lowUdpSend, &custom));
        LoopFunc loop_udpRecv("low_udp_recv", 0.002, 3, boost::bind(&Custom::lowUdpRecv, &custom));

        loop_udpSend.start();
        loop_udpRecv.start();

        printf("LOWLEVEL is initialized\n");

#ifdef USE_ROS1
        ros::spin();
#else
        rclcpp::spin(nh);
#endif

        // printf("low level runing!\n");
    }
    else if (strcasecmp(args[1].c_str(), "HIGHLEVEL") == 0)
    {
#ifdef USE_ROS1
        sub_high = nh.subscribe("high_cmd", 1, highCmdCallback);
        pub_high = nh.advertise<unitree_msgs::HighState>("high_state", 1);
#else
        sub_high = nh->create_subscription<unitree_msgs::HighCmd>("high_cmd", 1, highCmdCallback);
        pub_high = nh->create_publisher<unitree_msgs::HighState>("high_state", 1);
#endif

        LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
        LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

        loop_udpSend.start();
        loop_udpRecv.start();

        printf("HIGHLEVEL is initialized\n");

#ifdef USE_ROS1
        ros::spin();
#else
        rclcpp::spin(nh);
#endif

        // printf("high level runing!\n");
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel(not case sensitive)" << std::endl;
        exit(-1);
    }

#ifdef USE_ROS2
    rclcpp::shutdown();
#endif
    return 0;
}
