cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

# Check if compiler is GCC or Clang, then add extra warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable compile_commands.json for linting
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # For custom service generation
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(catch_ros2 REQUIRED)  # Add Catch2

# Enable custom service generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeString.srv"
  DEPENDENCIES std_msgs
)

# Build talker (publisher) node
add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs tf2 tf2_ros)

# Build listener (subscriber) node
add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs tf2 tf2_ros)

# ROS Interface dependencies for custom service
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(talker ${typesupport_target})

# Install binaries and launch files
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Catch2 Tests
if(BUILD_TESTING)
  include(CTest)
  # Skip linters
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)

  # Add Catch2 test executable
  set(TESTFILES
    test/test_talker.cpp)  # Specify the test file

  add_executable(test_talker ${TESTFILES})
  target_link_libraries(test_talker catch_ros2::catch_ros2_with_node_main ${typesupport_target})
  ament_target_dependencies(test_talker rclcpp std_msgs)
  
  install(TARGETS test_talker
    DESTINATION lib/${PROJECT_NAME})

  # Register the test with catch_ros2
  catch_ros2_add_integration_test(${PROJECT_NAME}_test LAUNCH_FILE test.launch.py)
endif()

ament_package()
