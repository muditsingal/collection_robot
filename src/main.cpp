/**
 * @file main.cpp
 * @author Abhimanyu Saxena (Abhimanyu Saxena)
 * @brief
 * @version 0.1
 * @date 2023-12-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <rclcpp/utilities.hpp>
#include "object_detector.hpp"
#include "handle_trash.hpp"

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    HandleTrash trash_obj;
    ObjectDetect detect_obj;

    // RCLCPP_INFO(rclcpp::get_logger(), "Starting Trash Collection");
    while (rclcpp::ok()) {
        if (detect_obj.find_block()) {
            break;
        }
    }
    rclcpp::sleep_for(1s);
    trash_obj.remove_trash();

    while (rclcpp::ok()) {
        if (detect_obj.find_block()) {
            break;
        }
    }
    rclcpp::sleep_for(1s);
    trash_obj.remove_trash();

    while (rclcpp::ok()) {
        if (detect_obj.find_block()) {
            break;
        }
    }
    rclcpp::sleep_for(1s);
    trash_obj.remove_trash();

    rclcpp::shutdown();

    return 0;
}