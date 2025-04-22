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

# Include the src directory
include_directories(${CMAKE_SOURCE_DIR}/src)

# Create an executable for your localisation node
add_executable(localisation_node src/localisation.cpp)
ament_target_dependencies(localisation_node rclcpp geometry_msgs nav2_msgs std_msgs)

add_executable(object_detector_node src/detection.cpp)
ament_target_dependencies(object_detector_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(object_detector_node ${OpenCV_LIBS})

# Install the executable
install(TARGETS localisation_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS object_detector_node
DESTINATION lib/${PROJECT_NAME})

