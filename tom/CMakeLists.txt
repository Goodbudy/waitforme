cmake_minimum_required(VERSION 3.5)
project(tom)  # Package name is "tom"

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include the src directory
include_directories(${CMAKE_SOURCE_DIR}/src)

# Create an executable for your localisation node
add_executable(localisation_node src/localisation.cpp)
ament_target_dependencies(localisation_node rclcpp geometry_msgs nav2_msgs std_msgs OpenCV sensor_msgs nav_msgs tf2 tf2_geometry_msgs visualization_msgs)

# Create an executable for the detection node
add_executable(detection_node src/detection.cpp)
ament_target_dependencies(detection_node rclcpp geometry_msgs nav2_msgs std_msgs OpenCV sensor_msgs nav_msgs tf2 tf2_geometry_msgs visualization_msgs)

# Link OpenCV for the detection node
target_link_libraries(detection_node ${OpenCV_LIBS})

# Install the executable for localisation node
install(TARGETS localisation_node
  DESTINATION lib/${PROJECT_NAME})

# Install the detection node executable
install(TARGETS detection_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files (if you have any)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
