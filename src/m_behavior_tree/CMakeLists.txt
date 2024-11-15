cmake_minimum_required(VERSION 3.8)
project(m_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# Add library for custom behavior tree nodes
add_library(bt_nodes src/bt_nodes.cpp)
ament_target_dependencies(bt_nodes behaviortree_cpp behaviortree_ros2 rclcpp std_msgs)

# Add executable for bt_executor and link it with bt_nodes
add_executable(bt_executor src/bt_executor.cpp)
target_link_libraries(bt_executor bt_nodes)
ament_target_dependencies(bt_executor behaviortree_cpp behaviortree_ros2 rclcpp std_msgs)

# Install targets
install(TARGETS
  bt_nodes
  bt_executor
  DESTINATION lib/${PROJECT_NAME}
)

# Testing setup (if using tests)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

