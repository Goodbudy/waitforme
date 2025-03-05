cmake_minimum_required(VERSION 3.5)
project(issy)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

# Declare the executable for movement logic
add_executable(movement_logic src/movementlogic.cpp)
ament_target_dependencies(movement_logic
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

# Declare the executable for ready_to_go_service
add_executable(ready_to_go_service src/readytogo_service.cpp)
ament_target_dependencies(ready_to_go_service
  rclcpp
  std_srvs
  std_msgs
  ament_cmake
)

# Declare the executable for goals_service
add_executable(goals_service src/goals_service.cpp)
ament_target_dependencies(goals_service
  rclcpp
  std_srvs
  std_msgs
  ament_cmake
)

# Install executables
install(TARGETS
  movement_logic
  ready_to_go_service
  goals_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
