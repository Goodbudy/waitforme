cmake_minimum_required(VERSION 3.5)
project(issy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(movementlogic src/movementlogic.cpp)
ament_target_dependencies(movementlogic rclcpp nav2_msgs rclcpp_action geometry_msgs std_msgs)

install(TARGETS
  movementlogic
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
