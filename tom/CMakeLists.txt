cmake_minimum_required(VERSION 3.5)
project(tom)  # Package name is "tom"

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include the src directory
include_directories(${CMAKE_SOURCE_DIR}/src)

# Create an executable for your localization node
add_executable(localization_node src/localization.cpp)
ament_target_dependencies(localization_node rclcpp geometry_msgs nav2_msgs std_msgs)

# Install the executable
install(TARGETS localization_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
