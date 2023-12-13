/**
 * @file ObjectDetect.hpp
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

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <apriltag.h>

using namespace std::chrono_literals;
using POSE_PUB = rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr;
using IMG_SUB = rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr;


class ObjectDetect : public rclcpp::Node {
  public:
    ObjectDetect();

  private:
    void timerCallback();

    void framesCallback();

    geometry_msgs::msg::Pose detectNearestObject();

    POSE_PUB nearest_object_pose;
    IMG_SUB read_frames_;
    sensor_msgs::msg::Image frames_;
    rclcpp::TimerBase timer_;

};
