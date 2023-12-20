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