cmake_minimum_required(VERSION 3.8)
project(m_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

# Create library for behavior tree nodes
add_library(bt_nodes src/bt_nodes.cpp)
ament_target_dependencies(bt_nodes behaviortree_cpp rclcpp)  # Link dependencies to bt_nodes library

# Create executable for behavior tree executor
add_executable(bt_executor src/bt_executor.cpp)
target_link_libraries(bt_executor bt_nodes)  # Link custom library
ament_target_dependencies(bt_executor behaviortree_cpp rclcpp)  # Link dependencies

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

