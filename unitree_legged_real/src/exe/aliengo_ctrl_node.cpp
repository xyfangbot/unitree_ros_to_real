#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#ifdef USE_ROS1
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#endif
#include "unitree_legged_sdk/unitree_legged_sdk.h"

namespace
{
volatile std::sig_atomic_t exit_requested = 0;
}

int main(int argc, char **argv)
{
    // Keep UDP alive through the sit/damp sequence after SIGINT or SIGTERM.
    std::signal(SIGINT, [](int) { exit_requested = 1; });
    std::signal(SIGTERM, [](int) { exit_requested = 1; });
#ifdef USE_ROS1
    ros::init(argc, argv, "aliengo_ctrl_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    std::string cmd_topic, target_ip;
    int local_port, target_port;
    private_nh.param<std::string>("cmd_topic", cmd_topic, "/cmd_planner");
    private_nh.param<std::string>("target_ip", target_ip, "192.168.123.220");
    private_nh.param("local_port", local_port, 8081);
    private_nh.param("target_port", target_port, 8082);
#else
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto nh = rclcpp::Node::make_shared("aliengo_ctrl_node");
    const auto cmd_topic = nh->declare_parameter<std::string>("cmd_topic", "/cmd_planner");
    const auto target_ip = nh->declare_parameter<std::string>("target_ip", "192.168.123.220");
    const auto local_port = nh->declare_parameter<int>("local_port", 8081);
    const auto target_port = nh->declare_parameter<int>("target_port", 8082);
#endif
    if (local_port < 1 || local_port > 65535 || target_port < 1 || target_port > 65535)
    {
        std::cerr << "UDP ports must be in [1, 65535]." << std::endl;
#ifdef USE_ROS1
        ros::shutdown();
#else
        rclcpp::shutdown();
#endif
        return 1;
    }

    using Clock = std::chrono::steady_clock;
    constexpr double timeout = 0.050;
    bool ready = false;
    bool stopping = false;
    bool valid_command = false;
    double command_stamp = 0.0;
    double last_accepted_stamp = 0.0;
    double forward_velocity = 0.0;
    double yaw_velocity = 0.0;
    Clock::time_point last_received;

    // Single-threaded ROS dispatch keeps callback data and control state serialized.
#ifdef USE_ROS1
    auto callback = [&](const geometry_msgs::TwistStamped::ConstPtr &msg)
#else
    auto callback = [&](const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg)
#endif
    {
        if (!ready || stopping)
            return;
#ifdef USE_ROS1
        const double now = ros::Time::now().toSec();
        const double stamp = msg->header.stamp.toSec();
#else
        const double now = nh->now().seconds();
        const double stamp = static_cast<double>(msg->header.stamp.sec) +
                             static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
#endif
        const double age = now - stamp;
        const double vx = msg->twist.linear.x;
        const double wz = msg->twist.angular.z;
        // Replayed stamps do not refresh the steady-clock watchdog. Invalid input
        // cancels the previous velocity rather than retaining it until timeout.
        valid_command = false;
        if (stamp <= 0.0 || stamp <= last_accepted_stamp || age < 0.0 || age > timeout ||
            !std::isfinite(vx) || !std::isfinite(wz) ||
            std::abs(vx) > std::numeric_limits<float>::max() ||
            std::abs(wz) > std::numeric_limits<float>::max())
            return;
        command_stamp = stamp;
        last_accepted_stamp = stamp;
        last_received = Clock::now();
        forward_velocity = vx;
        yaw_velocity = wz;
        valid_command = true;
    };
#ifdef USE_ROS1
    auto sub = nh.subscribe<geometry_msgs::TwistStamped>(cmd_topic, 1, callback);
#else
    auto sub = nh->create_subscription<geometry_msgs::msg::TwistStamped>(
        cmd_topic, rclcpp::QoS(1).best_effort().durability_volatile(), callback);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nh);
#endif

    using namespace UNITREE_LEGGED_SDK;
    // Bound Recv so the receive thread can always be joined on shutdown.
    UDP udp(static_cast<uint16_t>(local_port), target_ip.c_str(),
            static_cast<uint16_t>(target_port), sizeof(HighCmd), sizeof(HighState), 20);
    HighCmd cmd = {};
    udp.InitCmdData(cmd);
    std::atomic<bool> receiving{true};
    std::thread receiver([&] {
        while (receiving.load())
            udp.Recv();
    });

    struct Stage
    {
        uint8_t level;
        uint8_t mode;
        double seconds;
    };
    // Sport trigger: example_start_aliengo_sport.cpp (0xf0 for ~90 ms).
    // AlienGo state machine (motion program >= 1.0.17):
    // https://github.com/UnitreeSupport/Unitree_Docs/blob/master/docs/Aliengo/AlienGo.md
    // Stand: 7 -> 5 -> 6, unlock walking: 6 -> 1 -> 2.
    // Dwell times are conservative estimates based on example_end_aliengo_sport.cpp;
    // they issue transitions, and do not confirm physical posture completion.
    const std::array<Stage, 7> startup{{
        {HIGHLEVEL, 0, 0.010}, {0xf0, 0, 0.090}, {HIGHLEVEL, 0, 2.000},
        {HIGHLEVEL, 7, 0.500}, {HIGHLEVEL, 5, 1.800},
        {HIGHLEVEL, 6, 3.500}, {HIGHLEVEL, 1, 1.500}
    }};
    // Stop walking before crouching: 2 -> 1 -> 6 -> 5 -> 7.
    const std::array<Stage, 5> shutdown{{
        {HIGHLEVEL, 2, 0.200}, {HIGHLEVEL, 1, 1.500},
        {HIGHLEVEL, 6, 1.000}, {HIGHLEVEL, 5, 2.000}, {HIGHLEVEL, 7, 1.000}
    }};
    std::size_t stage_index = 0;
    auto stage_started = Clock::now();
    auto next_tick = stage_started;
    int result = 0;
    std::cout << "AlienGo: triggering sport mode, then standing with zero velocity. "
              << "Planner topic: " << cmd_topic << std::endl;

    while (true)
    {
#ifdef USE_ROS1
        const bool ros_ok = ros::ok();
#else
        const bool ros_ok = rclcpp::ok();
#endif
        if ((exit_requested || !ros_ok) && !stopping)
        {
            stopping = true;
            valid_command = false;
            // If interrupted while starting, do not stand up solely to sit down.
            if (cmd.mode == 5)
                stage_index = 3;
            else if (cmd.mode == 6)
                stage_index = 2;
            else if (cmd.mode == 1)
                stage_index = 1;
            else if (cmd.mode == 2)
                stage_index = 0;
            else
                stage_index = 4;
            stage_started = Clock::now();
            std::cout << "AlienGo: zero velocity, sit, then damping." << std::endl;
        }

        const auto now_steady = Clock::now();
        if (!ready || stopping)
        {
            const Stage *stages = stopping ? shutdown.data() : startup.data();
            const std::size_t count = stopping ? shutdown.size() : startup.size();
            if (std::chrono::duration<double>(now_steady - stage_started).count() >=
                stages[stage_index].seconds)
            {
                ++stage_index;
                // Start each dwell at actual entry, so scheduler delays cannot skip modes.
                stage_started = now_steady;
            }
            if (stage_index == count)
            {
                if (stopping)
                    break;
                ready = true;
                valid_command = false;
                std::cout << "AlienGo: walking enabled; waiting for a fresh planner command."
                          << std::endl;
            }
            else
            {
                cmd.levelFlag = stages[stage_index].level;
                cmd.mode = stages[stage_index].mode;
            }
        }

        // Do not service planner callbacks during shutdown. UDP continues even if
        // ROS has already shut down (e.g. rosnode kill or context shutdown).
        if (ros_ok && !stopping)
        {
            try
            {
#ifdef USE_ROS1
                ros::spinOnce();
#else
                executor.spin_some();
#endif
            }
            catch (const std::exception &error)
            {
                std::cerr << "ROS callback failure: " << error.what() << std::endl;
                exit_requested = 1;
                valid_command = false;
                result = 1;
            }
        }

        cmd.velocity[0] = 0.0f;
        cmd.velocity[1] = 0.0f;
        cmd.yawSpeed = 0.0f;
        cmd.position[0] = cmd.position[1] = 0.0f;
        cmd.rpy[0] = cmd.rpy[1] = cmd.rpy[2] = 0.0f;
        cmd.gaitType = 0;
        cmd.speedLevel = 0;
        cmd.dBodyHeight = 0.0f;
        cmd.dFootRaiseHeight = 0.0f;
        if (ready && !stopping)
        {
            cmd.levelFlag = HIGHLEVEL;
            cmd.mode = 2;
#ifdef USE_ROS1
            const double now_ros = ros::Time::now().toSec();
#else
            const double now_ros = nh->now().seconds();
#endif
            const double age = now_ros - command_stamp;
            if (valid_command && (age < 0.0 || age > timeout ||
                std::chrono::duration<double>(Clock::now() - last_received).count() > timeout))
                valid_command = false;
            if (valid_command && !exit_requested)
            {
                cmd.velocity[0] = static_cast<float>(forward_velocity);
                cmd.yawSpeed = static_cast<float>(yaw_velocity);
            }
        }
        udp.SetSend(cmd);
        udp.Send();
        // Wall/steady time keeps the watchdog and shutdown active if /clock stops.
        next_tick += std::chrono::milliseconds(2);
        if (next_tick < Clock::now())
            next_tick = Clock::now();
        std::this_thread::sleep_until(next_tick);
    }

    receiving.store(false);
    receiver.join();
#ifdef USE_ROS1
    ros::shutdown();
#else
    rclcpp::shutdown();
#endif
    return result;
}
