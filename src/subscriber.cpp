/**
 * @file subscriber.cpp
 * @brief This file contains the implementation of the MinimalSubscriber class.
 * @author Datta Lohith Gannavarapu
 * @version 2.0
 * @date 2024-11-06
 * @copyright Copyright (c) 2024
 * @license Apache 2.0
 */

#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @class MinimalSubscriber
 * @brief A simple ROS 2 node that subscribes to a custom topic.
 * @details Logs received messages at different severity levels.
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the MinimalSubscriber node.
   *        Initializes subscription to "custom_topic".
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    // DEBUG log indicating subscription initialization.
    RCLCPP_DEBUG(this->get_logger(),
      "Initializing subscription to 'custom_topic'");

    // Initialize subscription to "custom_topic" with a queue size of 10.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "custom_topic", 10,
        std::bind(&MinimalSubscriber::TopicCallback, this, _1));

    // INFO log indicating successful node startup.
    RCLCPP_INFO(this->get_logger(), "MinimalSubscriber node has started.");
  }

 private:
  /**
   * @brief Callback function for receiving messages.
   * @param msg The message received from the topic.
   */
  void TopicCallback(const std_msgs::msg::String &msg) const {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg.data.c_str());

    // Demonstrating WARN log when a specific condition is met.
    if (msg.data.empty()) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Received an empty message.");
    }

    // Demonstrating ERROR log for example purposes.
    if (msg.data == "error") {
      RCLCPP_ERROR(this->get_logger(),
        "Simulated error log on receiving 'error' string.");
    }

    // Demonstrating FATAL log if a critical string is detected.
    if (msg.data == "fatal") {
      RCLCPP_FATAL(this->get_logger(),
        "Received 'fatal' string, indicating critical error.");
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      subscription_;  ///< Subscription handle
};

/**
 * @brief Main function to initialize the ROS 2 node and start spinning.
 * @param argc Argument count.
 * @param argv Argument values.
 * @return int Program exit status.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // DEBUG log indicating the main function has started.
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
    "Initializing MinimalSubscriber node.");

  rclcpp::spin(std::make_shared<MinimalSubscriber>());

  // INFO log indicating the node is shutting down.
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
    "Shutting down MinimalSubscriber node.");

  rclcpp::shutdown();
  return 0;
}
