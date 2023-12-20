/**
 * @file object_detector.cpp
 * @author Mudit Singal (muditsingal@gmail.com)
 * @author Abhimanyu Saxena (asaxena4@umd.edu)
 * @brief C++ file for performing detections of blocks and publishing detections
 * @version 0.1
 * @date 2023-12-19
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "object_detector.hpp"
#include <functional>
#include <rclcpp/logging.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using std::placeholders::_1;

ObjectDetect::ObjectDetect() : Node("object_detector") {

    // Init the pub,sub and image processing pipeline
    image_node = rclcpp::Node::make_shared("image_listener", node_options);
    pub_velocity_msg = this->create_publisher<TWIST>("cmd_vel", 10);
    // Init odom subscriber
    odom_node = rclcpp::Node::make_shared("odom_node");
    odom_sub = odom_node->create_subscription<ODOM>("odom", 10,
        std::bind(&ObjectDetect::odometry_read_callback, this, _1));
    // init image publisher
    test_img_pub = this->create_publisher<IMAGE>("/contour_img", 10);
    trash_detector_pub = this->create_publisher<BOOL_>("/trash_found", 10);
}
