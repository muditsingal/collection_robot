/**
 * @file handle_trash.cpp
 * @author Mudit Singal (muditsingal@gmail.com)
 * @brief C++ file for exploring the workspace to detect blocks
 * @version 0.1
 * @date 2023-12-11
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "handle_trash.hpp"
#include <rclcpp/executors.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

HandleTrash::HandleTrash() : Node("handle_trash") {
  unspawn_client =
      create_client<gazebo_msgs::srv::DeleteEntity>("delete_entity");

  trash_handler_node = rclcpp::Node::make_shared("handle_trash");
  block_indices = {};
  blk_cntr = 3;
  for (int i = 0; i < blk_cntr; i++)
    block_indices.push_back(i);
}

bool HandleTrash::remove_trash() {
    while (!unspawn_client->wait_for_service(2s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(),
                                    "Interruped service call");
          return false;
        }
        RCLCPP_INFO(this->get_logger(), "Service currently not available... ");
    }

    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    int closest_blk = get_closest_block();
    if (closest_blk == -1) {
      RCLCPP_ERROR(this->get_logger(), "No blocks left to delete!");
      return false;
    }
    request->name = "trash_block" + std::to_string(closest_blk);

    auto srv_response = unspawn_client->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(trash_handler_node,
                                            srv_response, 15s);
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
        return true;
    } else {
        return false;
    }
}

int HandleTrash::get_closest_block() {
  if (block_indices.empty())
    return -1;
  int closest_idx = static_cast<int>(block_indices.front());
  block_indices.pop_front();
  return closest_idx;
}
