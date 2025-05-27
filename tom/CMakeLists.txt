cmake_minimum_required(VERSION 3.5)
project(tom)  # Package name is "tom"

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include the src directory
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Create an executable for your localisation node
add_executable(localisation_node src/localisation.cpp)
ament_target_dependencies(localisation_node rclcpp geometry_msgs nav2_msgs std_msgs OpenCV sensor_msgs nav_msgs tf2 tf2_geometry_msgs visualization_msgs)

#localisation hardcoded
add_executable(localisation_node_initial src/localisation_initial.cpp)
ament_target_dependencies(localisation_node_initial rclcpp geometry_msgs nav2_msgs std_msgs OpenCV sensor_msgs nav_msgs tf2 tf2_geometry_msgs visualization_msgs)

#localisation V3
add_executable(kalman_filter_node src/kalman_filter_node.cpp)
ament_target_dependencies(kalman_filter_node rclcpp nav_msgs Eigen3)

# Create an executable for the detection node
add_executable(detection_node src/detection.cpp)
ament_target_dependencies(detection_node yaml-cpp rclcpp geometry_msgs nav2_msgs std_msgs OpenCV sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs)

add_executable(people_detector src/people_detector.cpp)
ament_target_dependencies(people_detector rclcpp sensor_msgs visualization_msgs cv_bridge OpenCV)

# Link OpenCV for the detection node
target_link_libraries(detection_node yaml-cpp ${OpenCV_LIBS})


# Install the executable for localisation node
install(TARGETS localisation_node
  DESTINATION lib/${PROJECT_NAME})

# localisation Hardcoded
install(TARGETS localisation_node_initial
  DESTINATION lib/${PROJECT_NAME})

# Localisation V3
install(TARGETS kalman_filter_node
  DESTINATION lib/${PROJECT_NAME})

# Install the detection node executable
install(TARGETS detection_node
  DESTINATION lib/${PROJECT_NAME})

# install the people detector executable
install(TARGETS
people_detector
DESTINATION lib/${PROJECT_NAME}
)
# Install launch files (if you have any)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
