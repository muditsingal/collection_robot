/**
 * @file block_detector.hpp
 * @author Mudit Singal (muditsingal@gmail.com)
 * @brief Header file for performing and publishing detections of blocks
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

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <apriltag.h>

using namespace std::chrono_literals;

class block_detector : public rclcpp::Node {
  public:
    block_detector();
    

  private:

}
