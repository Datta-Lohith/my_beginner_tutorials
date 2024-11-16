/**
 * @file test_talker.cpp
 * @brief Level 2 integration test for the Talker node using Catch2.
 * @author
 * @version 1.0
 * @date 2024-11-15
 * @copyright Copyright (c) 2024
 * @license Apache 2.0
 */

#include <catch_amalgamated.hpp>
#include <chrono>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <catch_ros2/catch_ros2.hpp>
#include "beginner_tutorials/srv/change_string.hpp"

/**
  * @class TestNode
  * @brief A Test Fixture class for the Talker node.
  */
class TestNode : public rclcpp::Node {
 public:
  TestNode() : Node("test_node") {
    declare_parameter<double>("test_duration", 2.0);
    test_duration_ = get_parameter("test_duration").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Test duration: " << test_duration_);
  }

  ~TestNode() {
    RCLCPP_INFO_STREAM(get_logger(), "All tests completed.");
  }
 protected:
  double test_duration_;
};


TEST_CASE_METHOD(TestNode, "Validate publisher count on topic 'chatter'",
  "[talker]") {
  // Run the test for the specified duration
  auto startTime = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - startTime;
  auto timeout = rclcpp::Duration::from_seconds(test_duration_);
  while (rclcpp::ok() && duration < timeout) {
    rclcpp::spin_some(this->get_node_base_interface());
    duration = rclcpp::Clock().now() - startTime;

  // Create a publisher on the "chatter" topic
  auto test_publisher = this->create_publisher<std_msgs::msg::String>
    ("chatter", 10);

  // Check the number of publishers on the "chatter" topic
  auto publisher_count = this->count_publishers("chatter");

  // Validate that the publisher count is as expected
  REQUIRE(static_cast<int>(publisher_count) == 1);
  }
}

TEST_CASE_METHOD(TestNode, "Validate service availability for 'change_string'",
  "[service]") {
  // Run the test for the specified duration
  auto startTime = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - startTime;
  auto timeout = rclcpp::Duration::from_seconds(test_duration_);
  auto service_available = false;

  // Create a client for the "change_string" service
  auto test_client = this->create_client<beginner_tutorials::srv::ChangeString>
    ("change_message");
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Client created for 'change_message' service.");

  while (rclcpp::ok() && duration < timeout) {
    rclcpp::spin_some(this->get_node_base_interface());
    duration = rclcpp::Clock().now() - startTime;

    // Check the availability of the "change_string" service
    service_available = test_client->service_is_ready();
  }
  // Validate that the service is available
  CHECK(service_available);
}
