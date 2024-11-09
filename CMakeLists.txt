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

# Enable custom service generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeString.srv"
  DEPENDENCIES std_msgs
)

# Build talker (publisher) node
add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# Build listener (subscriber) node
add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# ROS Interface dependencies for custom service
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(talker ${typesupport_target})
# target_link_libraries(listener ${typesupport_target})

# Install binaries and launch files
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


# Google C++ Style and Lint checks
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  
  ament_lint_auto_find_test_dependencies()
endif()

# Check if cpplint executable is available
find_program(CPPLINT_EXECUTABLE cpplint)
if(CPPLINT_EXECUTABLE)
  # Run cpplint when configuring the project
  message(DEBUG " - Running cpplint on source files")
  execute_process(
    COMMAND ${CPPLINT_EXECUTABLE} --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/*.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_FILE ${CMAKE_BINARY_DIR}/results/cpplint.txt
    RESULT_VARIABLE CPPLINT_RESULT
    COMMAND echo "Running cpplint on source files"
  )
endif()

# Check if clang-tidy executable is available
find_program(CLANG_TIDY_EXECUTABLE clang-tidy)
if(CLANG_TIDY_EXECUTABLE)
  message(DEBUG " - Running clang-tidy on source files")
  # Run clang-tidy when configuring the project
  execute_process(
    COMMAND ${CLANG_TIDY_EXECUTABLE} --extra-arg=-stdlib=libc++ --extra-arg=-I/usr/include/c++/11 --extra-arg=-I/usr/include/x86_64-linux-gnu/c++/11 src/*.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_FILE ${CMAKE_BINARY_DIR}/results/clang-tidy.txt
    RESULT_VARIABLE CLANG_TIDY_RESULT
  )

endif()


ament_package()
