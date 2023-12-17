/**
 * @file object_detector.cpp
 * @author Mudit Singal (muditsingal@gmail.com)
 * @brief C++ file for performing detections of blocks and publishing detections
 * @version 0.1
 * @date 2023-12-11
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "object_detector.hpp"
using std::placeholders::_1;

ObjectDetect::ObjectDetect() : Node("object_detector") {
  /**
   * @brief Publisher object for publishing pose of nearest detected pose
   *
   */
  nearest_object_pose = this->create_publisher<POSE>(
    "/object_detector/nearest_pose", 10);

  /**
   * @brief Timer for publishing the messages with frequency of 10 Hz
   *
   */
  timer_ = this->create_wall_timer(
    100ms, std::bind(&ObjectDetect::nearest_object_pose, this));

  /**
   * @brief Subscriber object for subscribing to the image published by Camera
   * onboard the Turtlebot3 Waffle Pi
   */
  read_frames_ = this->create_subscription<IAMGE>(
  "/camera/image_raw", 10, std::bind(&ObjectDetect::framesCallback, this, _1));
}
