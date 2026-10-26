#include <iostream>
#ifdef USE_ROS1
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

unitree_msgs::HighState high_state_ros;

#ifdef USE_ROS1
void highStateCallback(const unitree_msgs::HighState::ConstPtr &state)
#else
void highStateCallback(const unitree_msgs::HighState::ConstSharedPtr &state)
#endif
{
    static long count = 0;
#ifdef USE_ROS1
    ROS_INFO("highStateCallback %ld", count++);
#else
    RCLCPP_INFO(rclcpp::get_logger("unitree_legged_real"), "highStateCallback %ld", count++);
#endif
    high_state_ros = *state;
}

int main(int argc, char **argv)
{
#ifdef USE_ROS1
    ros::init(argc, argv, "example_walk_without_lcm");
#else
    rclcpp::init(argc, argv);
#endif

    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

#ifdef USE_ROS1
    ros::NodeHandle nh;
#else
    auto nh = rclcpp::Node::make_shared("example_walk");
#endif

#ifdef USE_ROS1
    ros::Rate loop_rate(500);
#else
    rclcpp::WallRate loop_rate(500);
#endif

    long motiontime = 0;

    unitree_msgs::HighCmd high_cmd_ros;

#ifdef USE_ROS1
    ros::Publisher pub = nh.advertise<unitree_msgs::HighCmd>("high_cmd", 1000);
    ros::Subscriber sub = nh.subscribe("high_state", 1000, highStateCallback);
#else
    auto pub = nh->create_publisher<unitree_msgs::HighCmd>("high_cmd", 1000);
    auto sub = nh->create_subscription<unitree_msgs::HighState>("high_state", 1000, highStateCallback);
#endif

#ifdef USE_ROS1
    while (ros::ok())
#else
    while (rclcpp::ok())
#endif
    {
        std::cout << "motiontime:\t" << motiontime << " " << high_state_ros.imu.rpy[2] << "\n";

        motiontime += 2;

        high_cmd_ros.level_flag = HIGHLEVEL;

        high_cmd_ros.velocity[0] = 0.0f;
        high_cmd_ros.velocity[1] = 0.0f;
        high_cmd_ros.position[0] = 0.0f;
        high_cmd_ros.position[1] = 0.0f;
        high_cmd_ros.yaw_speed = 0.0f;

        high_cmd_ros.mode = 0;
        high_cmd_ros.rpy[0]  = 0;
        high_cmd_ros.rpy[1] = 0;
        high_cmd_ros.rpy[2] = 0;
        high_cmd_ros.gait_type = 0;
        high_cmd_ros.d_body_height = 0;
        high_cmd_ros.d_foot_raise_height = 0;


        if (motiontime == 2)
        {
            std::cout<<"begin sending commands."<<std::endl;
        }
        if (motiontime>1000 && motiontime <2000)
        {
            high_cmd_ros.mode = 1;    // mode 1: force stand, control robot orientation and height
            high_cmd_ros.rpy[0] = 0.7;
        }
        if (motiontime>2000 && motiontime <3000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.rpy[0] = 0.;
        }
        if (motiontime>3000 && motiontime <4000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.rpy[1] = 0.4;
        }
        if (motiontime>4000 && motiontime <5000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.rpy[1] = 0.;
        }
        if (motiontime>5000 && motiontime <6000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.rpy[2] = 0.5;
        }
        if (motiontime>6000 && motiontime <7000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.rpy[2] = 0;
        }
        if (motiontime>8000 && motiontime <9000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.d_body_height = 0.05;
        }
        if (motiontime>9000 && motiontime <10000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.d_body_height = -0.15;
        }
        if (motiontime>10000 && motiontime <11000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.d_body_height = 0.;
        }
        if (motiontime>11000 && motiontime <13000)
        {
            high_cmd_ros.mode = 2;    // mode 2: following tareget velocity in body frame
            high_cmd_ros.gait_type = 0; // trot gait
            high_cmd_ros.velocity[0] = 0.2;
            high_cmd_ros.velocity[1] = 0;
            high_cmd_ros.yaw_speed = 0.3;
        }
        if (motiontime>13000 && motiontime <15000)
        {
            high_cmd_ros.mode = 2;
            high_cmd_ros.gait_type = 0;
            high_cmd_ros.velocity[0] = -0.2;
            high_cmd_ros.velocity[1] = 0;
            high_cmd_ros.yaw_speed = -0.1;
            high_cmd_ros.d_foot_raise_height = 0.1; // increase swing height
        }
        if (motiontime> 15000 && motiontime <17000)
        {
            high_cmd_ros.mode = 0; // normal trot of gaitType 0 will stop automatically
        }
        if (motiontime>17000 && motiontime <23000)
        {
            high_cmd_ros.mode = 3; // mode 3: walk to target position in ground frame
            high_cmd_ros.gait_type = 0;
            high_cmd_ros.speed_level = 0; // adjust speedlevel
            high_cmd_ros.position[0] = 1;
            high_cmd_ros.position[1] = 0;
            high_cmd_ros.rpy[2] = 0;
            high_cmd_ros.d_body_height = -0.05; // lower body height
        }
        if (motiontime> 23000 && motiontime <25000)
        {
            high_cmd_ros.mode = 0; // normal trot of gaitType 0 will stop automatically
        }
        if (motiontime>25000 && motiontime <32000)
        {
            high_cmd_ros.mode = 2;
            high_cmd_ros.gait_type = 2; // stairs walking gait
            high_cmd_ros.velocity[0] = 0.1; // avoid using higher forward speed than 0.37m/s for stair climbing gait
            high_cmd_ros.velocity[1] = 0.;
            high_cmd_ros.yaw_speed = 0;
        }
        if (motiontime>32000 && motiontime <40000)
        {
            high_cmd_ros.mode = 2;
            high_cmd_ros.gait_type = 2;
            high_cmd_ros.velocity[0] = -0.1; // avoid using higher backward speed than -0.2m/s for stair climbing gait
            high_cmd_ros.velocity[1] = 0.;
            high_cmd_ros.yaw_speed = 0;
        }
        if (motiontime>40000 && motiontime <42000)
        {
            high_cmd_ros.mode = 1; // stairs walking gait will not automatically stop. change mode to force stand.
        }

        if (motiontime>42000 && motiontime <44000)
        {
            high_cmd_ros.mode = 5; // stand down
        }
        if (motiontime>44000 && motiontime <46000)
        {
            high_cmd_ros.mode = 6; // stand up
        }
        if (motiontime>46000 && motiontime <48000)
        {
            high_cmd_ros.mode = 5; // stand down
        }
        if (motiontime>48000 && motiontime <51000)
        {
            high_cmd_ros.mode = 7; // damping mode
        }
        if (motiontime>51000 && motiontime <53000)
        {
            high_cmd_ros.mode = 5; // stand down
        }
        if (motiontime>53000 && motiontime <55000)
        {
            high_cmd_ros.mode = 6; // stand up
        }
        if (motiontime>55000 && motiontime <57000)
        {
            high_cmd_ros.mode = 1; // force stand
        }
        if (motiontime>57000 && motiontime <65000)
        {
            high_cmd_ros.mode = 3; // mode 3: walk to target position in ground frame
            high_cmd_ros.gait_type = 0;
            high_cmd_ros.speed_level = 0; // adjust speedlevel
            high_cmd_ros.position[0] = 0;
            high_cmd_ros.position[1] = 0;
            high_cmd_ros.rpy[2] = 0;
            high_cmd_ros.d_body_height = 0;
        }

        

#ifdef USE_ROS1
        pub.publish(high_cmd_ros);
        ros::spinOnce();
#else
        pub->publish(high_cmd_ros);
        rclcpp::spin_some(nh);
#endif
        loop_rate.sleep();
    }

#ifdef USE_ROS2
    rclcpp::shutdown();
#endif
    return 0;
}