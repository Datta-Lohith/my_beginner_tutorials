/**
 * @file publisher.cpp
 * @brief This file contains the implementation of the MinimalPublisher class.
 * @author Datta Lohith Gannavarapu
 * @version 2.0
 * @date 2024-11-06
 * @copyright Copyright (c) 2024
 */


#include <ctime>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/change_string.hpp"  // Include the service header

using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @class MinimalPublisher
 * @brief A simple ROS 2 node that publishes messages to a topic and provides a service to change the message.
 * 
 * This node initializes a publisher to the "custom_topic" and publishes a message that includes the current time.
 * It also provides a service "change_message" that allows the message content to be changed dynamically.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the MinimalPublisher node.
   * 
   * Initializes the publisher on the "custom_topic" and the service to change the message content.
   * A timer is set up to publish a message every 500 ms.
   */
  MinimalPublisher() : Node("minimal_publisher"),
    message_content_("My name is Datta Lohith Gannavarapu.") {
    // Initialize publisher on 'custom_topic' with a queue size of 10.
    publisher_ =
      this->create_publisher<std_msgs::msg::String>("custom_topic", 10);

    // Declare and get the parameter for message content
    this->declare_parameter<std::string>("message_content", message_content_);
    this->get_parameter("message_content", message_content_);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Message content: " <<
      message_content_);

    // Log an error if the message content is the default value
    if (message_content_ == "My name is Datta Lohith Gannavarapu.") {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Message content is default.");
    }

    // Initialize a timer that triggers the publishing callback every 500 ms.
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MinimalPublisher::TimerCallback, this));

    // Create the change message service
    change_message_service_ =
    this->create_service<beginner_tutorials::srv::ChangeString>(
      "change_message",
        std::bind(&MinimalPublisher::ChangeMessageService, this, _1, _2));

    RCLCPP_INFO_STREAM(this->get_logger(),
      "MinimalPublisher node initialized.");
  }

 private:
  /**
   * @brief Callback function that publishes the message with the current time.
   * 
   * This callback is triggered every 500 ms by the timer. It creates a message 
   * with the current time and publishes it to the "custom_topic".
   */
  void TimerCallback() {
    auto message = std_msgs::msg::String();
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    message.data = message_content_ + " Current time: " +
      std::string(std::ctime(&time));
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " +
      message.data);
    publisher_->publish(message);
  }

  /**
   * @brief Service callback that changes the message content.
   * 
   * This service callback allows the message content to be changed. It sets
   * the new message based on the incoming request and logs the updated message.
   * 
   * @param request The service request containing the new message content.
   * @param response The service response indicating success.
   */
  void ChangeMessageService(
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
      request,
      std::shared_ptr<beginner_tutorials::srv::ChangeString::Response>
      response) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Service request received.");
    message_content_ = request->message;
    response->success = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "Message content updated to: "
      + message_content_);
  }

  // Timer to trigger the publisher
  rclcpp::TimerBase::SharedPtr timer_;

  // Publisher handle for publishing messages to 'custom_topic'
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  /**
   * @brief Service handle for the 'change_message' service.
   * 
   * This handle allows the service to be called to update the message content.
   */
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr
    change_message_service_;

  // The current message content
  std::string message_content_;
};

/**
 * @brief Main function to initialize and run the MinimalPublisher node.
 * 
 * This function initializes the ROS 2 system, spins the node, and shuts it down
 * once the node has finished executing.
 * 
 * @param argc The argument count.
 * @param argv The argument values.
 * @return int The exit status.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
