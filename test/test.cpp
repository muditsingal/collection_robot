/**
 * @file test.cpp
 * @author Abhishekh Reddy (areddy@umd.edu)
 * @author Tommy Chang (chang177@umd.edu)
 * @brief Test cases for various nodes used in the collection_robot package.
 * @version 0.5
 * @date 2023-12-12
 *
 * @copyright Copyright (c) 2023 Abhishekh Reddy, Tommy Chang
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <stdlib.h>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture()
      : node_{std::make_shared<rclcpp::Node>("collection_robot_test")} {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Constructor initialized...");
  }

  void SetUp() override {
    bool retVal  {
      StartROSExec("collection_robot", "move_robot", "move_robot")
      && StartROSExec("collection_robot", "object_detect", "object_detect")
      && StartROSExec("collection_robot", "plan_path", "plan_path")};

    ASSERT_TRUE(retVal);

    RCLCPP_INFO_STREAM(node_->get_logger(), "Setup complete...");
  }

  void TearDown() override {
    bool retVal {StopROSExec()};
    ASSERT_TRUE(retVal);

    RCLCPP_INFO_STREAM(node_->get_logger(), "Teardown complete...");
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  std::stringstream cmd_ss, cmdInfo_ss, killCmd_ss;

  bool StartROSExec(const char* pkg_name, const char* node_name,
                                                        const char* exec_name) {
    // build command strings
    cmd_ss << "ros2 run " << pkg_name << " " << exec_name
                                               << " > /dev/null 2> /dev/null &";

    cmdInfo_ss << "ros2 node info " << "/" << node_name
                                                 << " > /dev/null 2> /dev/null";

    // pkill uses exec name <= 15 char only
    char execName[16];  snprintf(execName, sizeof(execName), "%s", exec_name);

    killCmd_ss << "pkill --signal SIGINT " << execName
                                                 << " > /dev/null 2> /dev/null";

    // First kill the ros2 node, in case it's still running.
    StopROSExec();

    // Start a ros2 node and wait for it to get ready:
    int retVal =  system(cmd_ss.str().c_str());
    if (retVal != 0)
      return false;

    retVal = -1;
    while (retVal != 0) {
      retVal = system(cmdInfo_ss.str().c_str());
      sleep(1);
    }
    return true;
  }

  bool StopROSExec() {
    if (killCmd_ss.str().empty())
      return true;

    int retVal =  system(killCmd_ss.str().c_str());
    return retVal == 0;
  }
};

TEST_F(TaskPlanningFixture, TrueIsTrueTest) {
  std::cout << "Starting test..." << std::endl;
  EXPECT_TRUE(true);

  using TWIST_SUBSCRIBER =
                     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr;
  using POSE_SUBSCRIBER =
                      rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr;
  bool hasData {false};
  bool hasTwistData {false};
  bool hasPoseData {false};

  TWIST_SUBSCRIBER twist_subscription =
      node_->create_subscription<geometry_msgs::msg::Twist>
    ("/cmd/vel", 10,
     // Lambda expression begins
     [&](const geometry_msgs::msg::Twist& msg) {
       RCLCPP_INFO(node_->get_logger(), "Received twist message");
       hasTwistData = true;
     });  // end of lambda expression

  POSE_SUBSCRIBER pose_subscription =
      node_->create_subscription<geometry_msgs::msg::Pose>
    ("/object_detect/nearest_objects", 10,
     // Lambda expression begins
     [&](const geometry_msgs::msg::Pose& msg) {
       RCLCPP_INFO(node_->get_logger(), "Received pose message");
       hasPoseData = true;
     });  // end of lambda expression

  /*
   * 3.) check to see if we get data winhin 3 sec
   */
  using timer = std::chrono::system_clock;
  using namespace std::chrono_literals;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);       // 2hz checks
  while (elapsed_time < 3s) {
      rclcpp::spin_some(node_);
      rate.sleep();
      elapsed_time = timer::now() - clock_start;
  }

  EXPECT_TRUE(hasData);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "Tests done, shutting down..." << std::endl;
  return result;
  }
