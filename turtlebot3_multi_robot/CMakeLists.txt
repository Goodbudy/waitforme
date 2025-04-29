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

# Add C++ executables
add_executable(Utest_single_goal_distribution src/Utest_single_goal_distribution.cpp src/TurtleBot.cpp src/GoalManager.cpp)
ament_target_dependencies(Utest_single_goal_distribution
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  rclcpp_action
)

# Install C++ executable
install(TARGETS
  Utest_single_goal_distribution
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(Utest_goal_reassignment src/Utest_goal_reassignment.cpp src/TurtleBot.cpp src/GoalManager.cpp)
ament_target_dependencies(Utest_goal_reassignment
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  rclcpp_action
)
install(TARGETS Utest_goal_reassignment DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch models rviz urdf worlds params
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multi_robot.dsv.in")

# Linting (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

