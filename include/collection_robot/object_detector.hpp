/**
 * @file object_detector.hpp
 * @author Mudit Singal (muditsingal@gmail.com)
 * @author Abhimanyu Saxena (asaxena4@umd.edu)
 * @brief Header file for performing and publishing detections of blocks
 * @version 0.1
 * @date 2023-12-19
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
#include <iomanip>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;
using TWIST = geometry_msgs::msg::Twist;
using std::placeholders::_1;
using std::chrono::duration;
using ODOM = nav_msgs::msg::Odometry;
using POSE = geometry_msgs::msg::Pose;
using IMAGE = sensor_msgs::msg::Image;
using BOOL_ = std_msgs::msg::Bool;

class ObjectDetect : public rclcpp::Node {
 private:
  cv::Mat image_feed_msg;
  sensor_msgs::msg::LaserScan lidar_feed_msg;
  rclcpp::NodeOptions node_options;
  image_transport::Subscriber image_sub;
  rclcpp::Node::SharedPtr image_node;
  rclcpp::Publisher<TWIST>::SharedPtr pub_velocity_msg;
  rclcpp::Publisher<IMAGE>::SharedPtr test_img_pub;
  rclcpp::Publisher<BOOL_>::SharedPtr trash_detector_pub;
  std_msgs::msg::Bool trash_detector_msg;
  sensor_msgs::msg::Image::SharedPtr test_img_msg;

  rclcpp::Node::SharedPtr odom_node;
  rclcpp::Subscription<ODOM>::SharedPtr odom_sub;
  bool turn_right;
  bool turn_left;
  bool go_forward;
  bool brake;
  bool trash_found;
  bool new_xyz;
  double current_angle;
  double initial_angle;

 public:
/**
 * @brief Construct a new Object Detect object
 *
 */
  ObjectDetect();

  /**
   * @brief Function to go to the detected block
   *
   * @return true If turtlebot is in vicinity of the block
   * @return false If turtlebot is not in vicinity of the block
   */
  bool go_to_block();

  /**
   * @brief Function to find the blue block
   *
   * @return true If a blue block is found
   * @return false If a blue block is not found
   */
  bool find_block();

  /**
   * @brief Function to subscribe image data from turtlebot
   *
   * @param image_data Image data
   */
  void image_read_callback(const IMAGE::ConstSharedPtr& image_data);

  /**
   * @brief Function to subscribe to odometry data
   *
   * @param odom_data Odom data
   */
  void odometry_read_callback(const ODOM::SharedPtr odom_data);
};
