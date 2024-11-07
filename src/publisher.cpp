/**
 * @file publisher.cpp
 * @brief This file contains the implementation of the MinimalPublisher class.
 * @author Datta Lohith Gannavarapu
 * @version 1.0
 * @date 2024-11-06
 * @copyright Copyright (c) 2024
 */

#include <ctime>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// MinimalPublisher class publishes custom messages to a topic.
class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher") {
    // Initialize publisher on 'custom_topic' with a queue size of 10.
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("custom_topic", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MinimalPublisher::TimerCallback, this));
  }

 private:
  // Publishes the current time as a string message on each timer tick.
  void TimerCallback() {
    auto message = std_msgs::msg::String();
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    message.data = "My name is Datta Lohith Gannavarapu. My age is "
                                  + std::string(std::ctime(&time));
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;  // Timer that triggers the publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      publisher_;  // Publisher handle
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
