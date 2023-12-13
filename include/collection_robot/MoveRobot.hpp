/**
 * @file block_detector.hpp
 * @author Mudit Singal (muditsingal@gmail.com)
 * @brief header file for exploring the workspace to detect blocks
 * @version 0.1
 * @date 2023-12-10
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using POSE_PUB = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr;
using namespace std::chrono_literals;

class MoveRobot : public rclcpp::Node {
  public:
    MoveRobot();

    bool explore_random();

    bool check_collision();



  private:
    POSE_PUB odom_pose_pub;


};
