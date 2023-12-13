/**
 * @file PlanPath.hpp
 * @author Mudit Singal (muditsingal@gmail.com)
 * @brief header file for planning paths to blocks and executing them
 * @version 0.1
 * @date 2023-12-10
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using POSEA_PUB = rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr;

class PlanPath : public rclcpp::Node {
  public:
    PlanPath();

  private:
    void moveTargetCallback();
    geometry_msgs::msg::PoseArray planPath();
    geometry_msgs::msg::Pose current_pose_, next_pose_;
    rclcpp::TimerBase timer_;
    POSEA_PUB move_target;

};
