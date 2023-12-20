/**
 * @file handle_trash.hpp
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
#include <list>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <gazebo_msgs/srv/detail/delete_entity__struct.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/laser_scan.hpp>


using TWIST_PUB = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;
using POSE_SUB_PTR = rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr;
using LASER_SUB_PTR = rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr;
using TIMER_PTR = rclcpp::TimerBase::SharedPtr;
using namespace std::chrono_literals;

/**
 * @brief Class of Move robot that handles tasks related to moving the robot
 * in the workspace
 *
 */
class HandleTrash : public rclcpp::Node {

  private:

    sensor_msgs::msg::LaserScan scan_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr unspawn_client;
    TIMER_PTR timer_;
    rclcpp::Node::SharedPtr trash_handler_node;
    int blk_cntr;
    std::list<int> block_indices {};
};
