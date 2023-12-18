/**
 * @file move_robot.cpp
 * @author Mudit Singal (muditsingal@gmail.com)
 * @brief C++ file for exploring the workspace to detect blocks
 * @version 0.1
 * @date 2023-12-11
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "move_robot.hpp"

MoveRobot::MoveRobot() : Node("move_robot") {
  timer_ = this->create_wall_timer(
      500ms,
      std::bind(&MoveRobot::timerCallback, this));

  mover_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  scanner_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "topic_name", // TODO: Add topic name here
      10,
      std::bind(&scannerCallback, this));
}
