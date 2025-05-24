cmake_minimum_required(VERSION 3.5)
project(turtlebot3_multi_robot)

# Ensure that the C++ standard is set
set(CMAKE_CXX_STANDARD 14)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(issy REQUIRED)

# Add executables
add_executable(Utest_single_goal_distribution
  src/Utest_single_goal_distribution.cpp
  src/TurtleBot.cpp
  src/GoalManager.cpp
)
ament_target_dependencies(Utest_single_goal_distribution
  rclcpp std_msgs geometry_msgs nav_msgs nav2_msgs rclcpp_action issy
)

add_executable(Utest_goal_reassignment
  src/Utest_goal_reassignment.cpp
  src/TurtleBot.cpp
  src/GoalManager.cpp
)
ament_target_dependencies(Utest_goal_reassignment
  rclcpp std_msgs geometry_msgs nav_msgs nav2_msgs rclcpp_action issy
)

add_executable(Utest_optimized_goal_assignment
  src/Utest_optimized_goal_assignment.cpp
  src/TurtleBot.cpp
  src/GoalManager.cpp
)
ament_target_dependencies(Utest_optimized_goal_assignment
  rclcpp std_msgs geometry_msgs nav_msgs nav2_msgs rclcpp_action issy
)

add_executable(manager
  src/managermain.cpp
  src/GoalManager.cpp
)
ament_target_dependencies(manager rclcpp std_msgs geometry_msgs nav_msgs nav2_msgs rclcpp_action issy)

# Install executables
install(TARGETS
  manager
  Utest_single_goal_distribution
  Utest_goal_reassignment
  Utest_optimized_goal_assignment
  DESTINATION lib/${PROJECT_NAME}
)

# Install resources (non-code)
install(DIRECTORY launch models rviz urdf worlds params
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multi_robot.dsv.in")

ament_package()
