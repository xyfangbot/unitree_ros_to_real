#include <algorithm>
#include <cmath>
#include <iostream>
#ifdef USE_ROS1
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

unitree_msgs::LowState low_state_ros;

#ifdef USE_ROS1
void lowStateCallback(const unitree_msgs::LowState::ConstPtr &state)
#else
void lowStateCallback(const unitree_msgs::LowState::ConstSharedPtr &state)
#endif
{
    static long count = 0;
#ifdef USE_ROS1
    ROS_INFO("lowStateCallback %ld", count++);
#else
    RCLCPP_INFO(rclcpp::get_logger("unitree_legged_real"), "lowStateCallback %ld", count++);
#endif

    low_state_ros = *state;
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}



int main(int argc, char **argv)
{
#ifdef USE_ROS1
    ros::init(argc, argv, "example_postition_without_lcm");
#else
    rclcpp::init(argc, argv);
#endif

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

#ifdef USE_ROS1
    ros::NodeHandle nh;
#else
    auto nh = rclcpp::Node::make_shared("example_position");
#endif
#ifdef USE_ROS1
    ros::Rate loop_rate(500);
#else
    rclcpp::WallRate loop_rate(500);
#endif

    float qInit[3] = {0};
    float qDes[3] = {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002;

    unitree_msgs::LowCmd low_cmd_ros;

    // bool initiated_flag = false; // initiate need time
    // int count = 0;

#ifdef USE_ROS1
    ros::Publisher pub = nh.advertise<unitree_msgs::LowCmd>("low_cmd", 1);
    ros::Subscriber sub = nh.subscribe("low_state", 1, lowStateCallback);
#else
    auto pub = nh->create_publisher<unitree_msgs::LowCmd>("low_cmd", 1);
    auto sub = nh->create_subscription<unitree_msgs::LowState>("low_state", 1, lowStateCallback);
#endif

    low_cmd_ros.level_flag = LOWLEVEL;

    for (std::size_t i = 0; i < 20; ++i)
    {
        low_cmd_ros.motor_cmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
        low_cmd_ros.motor_cmd[i].q = PosStopF; // 禁止位置环
        low_cmd_ros.motor_cmd[i].kp = 0;
        low_cmd_ros.motor_cmd[i].dq = VelStopF; // 禁止速度环
        low_cmd_ros.motor_cmd[i].kd = 0;
        low_cmd_ros.motor_cmd[i].tau = 0;
    }

#ifdef USE_ROS1
    while (ros::ok())
#else
    while (rclcpp::ok())
#endif
    {

        printf("FR_joint_pos: %f %f %f\n", low_state_ros.motor_state[FR_0].q, low_state_ros.motor_state[FR_1].q, low_state_ros.motor_state[FR_2].q);

        
        motiontime += 2;

        low_cmd_ros.motor_cmd[FR_0].tau = -1.6f;
   
        if (motiontime >= 0)
        {
        // first, get record initial position
        // if( motiontime >= 100 && motiontime < 500){
            if (motiontime >= 0 && motiontime < 10)
            {
                qInit[0] = low_state_ros.motor_state[FR_0].q;
                qInit[1] = low_state_ros.motor_state[FR_1].q;
                qInit[2] = low_state_ros.motor_state[FR_2].q;
            }
            // second, move to the origin point of a sine movement with Kp Kd
            // if( motiontime >= 500 && motiontime < 1500){
            if (motiontime >= 10 && motiontime < 400)
            {
                rate_count++;
                double rate = rate_count / 200.0; // needs count to 200
            // Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0;
            // Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
                Kp[0] = 20.0;
                Kp[1] = 20.0;
                Kp[2] = 20.0;
                Kd[0] = 2.0;
                Kd[1] = 2.0;
                Kd[2] = 2.0;

                qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
                qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
                qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
            }
            double sin_joint1, sin_joint2;
        // last, do sine wave
            float freq_Hz = 1;
        // float freq_Hz = 5;
            float freq_rad = freq_Hz * 2 * M_PI;
            float t = dt * sin_count;
            if (motiontime >= 400)
            {
                sin_count++;
            // sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
            // sin_joint2 = -0.9 * sin(3*M_PI*sin_count/1000.0);
                sin_joint1 = 0.6 * sin(t * freq_rad);
                sin_joint2 = -0.9 * sin(t * freq_rad);
                qDes[0] = sin_mid_q[0];
                qDes[1] = sin_mid_q[1] + sin_joint1;
                qDes[2] = sin_mid_q[2] + sin_joint2;
            // qDes[2] = sin_mid_q[2];
            }

            low_cmd_ros.motor_cmd[FR_0].q = qDes[0];
            low_cmd_ros.motor_cmd[FR_0].dq = 0;
            low_cmd_ros.motor_cmd[FR_0].kp = Kp[0];
            low_cmd_ros.motor_cmd[FR_0].kd = Kd[0];
            low_cmd_ros.motor_cmd[FR_0].tau = -1.6f;

            low_cmd_ros.motor_cmd[FR_1].q = qDes[1];
            low_cmd_ros.motor_cmd[FR_1].dq = 0;
            low_cmd_ros.motor_cmd[FR_1].kp = Kp[1];
            low_cmd_ros.motor_cmd[FR_1].kd = Kd[1];
            low_cmd_ros.motor_cmd[FR_1].tau = 0.0f;

            low_cmd_ros.motor_cmd[FR_2].q = qDes[2];
            low_cmd_ros.motor_cmd[FR_2].dq = 0;
            low_cmd_ros.motor_cmd[FR_2].kp = Kp[2];
            low_cmd_ros.motor_cmd[FR_2].kd = Kd[2];
            low_cmd_ros.motor_cmd[FR_2].tau = 0.0f;
        }
        

#ifdef USE_ROS1
        pub.publish(low_cmd_ros);
        ros::spinOnce();
#else
        pub->publish(low_cmd_ros);
        rclcpp::spin_some(nh);
#endif
        loop_rate.sleep();
    }

#ifdef USE_ROS2
    rclcpp::shutdown();
#endif
    return 0;
}
