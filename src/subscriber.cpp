/**
 * @file subscriber.cpp
 * @brief This file contains the implementation of the MinimalSubscriber class.
 * @author Datta Lohith Gannavarapu
 * @version 1.0
 * @date 2024-11-06
 * @copyright Copyright (c) 2024
 */

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

// MinimalSubscriber class subscribes to the "custom_topic" topic to receive
// time-based messages.
class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    // Initialize subscription to "custom_topic" with a queue size of 10.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "custom_topic", 10,
        std::bind(&MinimalSubscriber::TopicCallback, this, _1));
  }

 private:
  // Callback function that logs the received message.
  void TopicCallback(const std_msgs::msg::String &msg) const {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg.data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      subscription_;  // Subscription handle
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
