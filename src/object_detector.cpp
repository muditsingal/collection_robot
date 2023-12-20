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


// Function to go to the detected block
bool ObjectDetect::go_to_block() {
    auto bot = TWIST();
    if (turn_right) {
        bot.angular.z = -0.05;
        bot.linear.x = 0;
    } else if (turn_left) {
        bot.angular.z = 0.05;
        bot.linear.x = 0;
    } else if (go_forward) {
        bot.linear.x = 0.1;
        bot.angular.z = 0;
    } else if (brake) {
        bot.linear.x = 0;
        bot.angular.z = 0;
    }
    pub_velocity_msg->publish(bot);
    return true;
}

// Function to subscribe image data from turtlebot
void ObjectDetect::image_read_callback(const
            sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {

        cv::Mat hsv, thr, bin;
        cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;
        int H_min = 100, S_min = 80, V_min = 80;
        int H_max = 132, S_max = 255, V_max = 255;
        cv::cvtColor(image, hsv, CV_RGB2HSV);

        cv::inRange(hsv,
            cv::Scalar(H_min, S_min, V_min),
            cv::Scalar(H_max, S_max, V_max), thr);

        cv::threshold(thr, bin, 100, 255, cv::THRESH_BINARY);

        trash_detector_msg.data = trash_found;
        trash_detector_pub->publish(trash_detector_msg);
        trash_found = false;

        std::vector<std::vector<cv::Point> > contours;
        cv::Mat contourMat = thr.clone();
        cv::findContours(contourMat, contours, CV_RETR_LIST,
                    CV_CHAIN_APPROX_NONE);
        cv::Mat contourImage(image.size(), CV_8UC3,
                    cv::Scalar(0, 0, 0));
        cv::Scalar colors[3];
        colors[0] = cv::Scalar(255, 0, 0);
        colors[1] = cv::Scalar(0, 255, 0);
        colors[2] = cv::Scalar(0, 0, 255);
        if (contours.size() > 0) {
            trash_found = true;
            for (size_t idx = 0; idx < contours.size(); idx++) {
                cv::drawContours(contourImage, contours,
                        idx, colors[idx % 3]);
            }

            cv::Rect bbox;
            bbox = cv::boundingRect(contours.at(0));
            int cx = static_cast<int>((bbox.x+bbox.width)/2);
            int area = static_cast<int>(bbox.area());

            if (cx < 180) {
                turn_left = true;
                turn_right = false;
            } else if (cx > 200) {
                    turn_right = true;
                    turn_left = false;
            } else {
                if (area > 40000) {
                    brake = true;
                    go_forward = false;
                } else {
                    go_forward = true;
                    brake = false;
                }
                turn_left = false;
                turn_right = false;
            }
        } else {
            if (current_angle - initial_angle > 1.57) {
                new_xyz = true;
                turn_left = false;
                turn_right = false;
                go_forward = false;
                brake = true;
            } else {
                turn_left = true;
                turn_right = false;
                go_forward = false;
                brake = false;
            }
        }

    } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(),
            "Error converting ImgMsg to cv Image");
    }
}

// Function to find the blue block
bool ObjectDetect::find_block() {
    RCLCPP_INFO(this->get_logger(), "Finding Trash Blocks");
    rclcpp::spin_some(odom_node);

    initial_angle = current_angle;
    turn_right = false;
    turn_left = false;
    go_forward = false;
    brake = false;
    new_xyz = false;
    trash_found = false;

    // subscribe to image topic from trutlebot3
    image_transport::ImageTransport it(image_node);
    image_sub = it.subscribe("camera/image_raw", 1,
        std::bind(&ObjectDetect::image_read_callback, this, _1));


    while (true) {

        // run both odom and image nodes
        rclcpp::spin_some(odom_node);
        rclcpp::spin_some(image_node);
        if (brake) {
            go_to_block();
            break;
        } else if (new_xyz) {
            go_to_block();
            return false;
        } else {
            go_to_block();
            rclcpp::sleep_for(500ms);
        }
    }
    return true;
}