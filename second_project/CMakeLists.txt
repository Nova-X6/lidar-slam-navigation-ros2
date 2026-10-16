cmake_minimum_required(VERSION 3.8)
project(second_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ── goal_publisher ────────────────────────────────────────────────────────────
add_executable(goal_publisher src/goal_publisher.cpp)
ament_target_dependencies(goal_publisher
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

# ── pointcloud_to_laserscan (built-in, no external package needed) ────────────
add_executable(pointcloud_to_laserscan src/pointcloud_to_laserscan.cpp)
ament_target_dependencies(pointcloud_to_laserscan
  rclcpp
  sensor_msgs
)

install(TARGETS
  goal_publisher
  pointcloud_to_laserscan
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY csv     DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY map     DESTINATION share/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
