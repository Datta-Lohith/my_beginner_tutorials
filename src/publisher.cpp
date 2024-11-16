/**
 * @file publisher.cpp
 * @brief This file contains the implementation of the MinimalPublisher class.
 * @author Datta Lohith Gannavarapu
 * @version 3.0
 * @date 2024-11-15
 * @copyright Copyright (c) 2024
 * @license Apache 2.0
 */

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <ctime>
#include <memory>
#include <string>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "beginner_tutorials/srv/change_string.hpp"  // Include the service header

using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @class MinimalPublisher
 * @brief A ROS 2 node that publishes messages to a topic, broadcasts a static transform, 
 * and provides a service to change the message.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the MinimalPublisher node.
   * 
   * Initializes the publisher on "custom_topic", the "change_message" service, and the static transform broadcaster.
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

    // Initialize the static transform broadcaster
    tf_static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    RCLCPP_INFO_STREAM(this->get_logger(),
      "MinimalPublisher node initialized.");
  }

 private:
  /**
   * @brief Timer callback that publishes the message and broadcasts a static transform.
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

    // Broadcast the static transform
    broadcast_transform();
  }

  /**
   * @brief Service callback that changes the message content.
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
      << message_content_);
  }

  /**
   * @brief Broadcasts a static transform between the "world" and "talk" frames.
   */
  void broadcast_transform() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "world";
    transform.child_frame_id = "talk";

    // Translation in meters
    transform.transform.translation.x = 0.2;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.6;

    // Orientation (quaternion)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0473595, 0.0);  // Roll, pitch, yaw in radians
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(transform);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr
    change_message_service_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::string message_content_;
};

/**
 * @brief Main function to initialize and run the MinimalPublisher node.
 * 
 * @param argc Argument count.
 * @param argv Argument values.
 * @return int Exit status.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
