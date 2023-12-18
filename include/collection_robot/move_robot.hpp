/**
 * @file move_robot.hpp
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

using namespace std::chrono_literals;

/**
 * @brief Class of Move robot that handles tasks related to moving the robot
 * in the workspace
 *
 */
class MoveRobot : public rclcpp::Node {
  public:
    MoveRobot();

  private:
    void timerCallback();

    void scannerCallback();

    bool robotSense(char direction);

    bool moveTo(const geometry_msgs::msg::Pose target_pose);

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr objects_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mover_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanner_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan scan_;
};
