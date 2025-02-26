cmake_minimum_required(VERSION 3.5)
project(issy)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Create executable
add_executable(proofofconcept src/proofofconcept.cpp)

# Link dependencies
ament_target_dependencies(proofofconcept
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Correct installation (target name only, no path)
install(TARGETS proofofconcept
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()
