cmake_minimum_required(VERSION 3.8)
project(hallie)

# Ensure that the C++ standard is set
set(CMAKE_CXX_STANDARD 14)

# Check for compiler flags
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

# Add your executable (replace with your actual source files)
add_executable(Utest_single_goal_distribution src/Utest_single_goal_distribution.cpp src/TurtleBot.cpp src/GoalManager.cpp) #src/MainExe_Utest.cpp)

# Link dependencies to your executable
ament_target_dependencies(Utest_single_goal_distribution rclcpp std_msgs geometry_msgs nav_msgs nav2_msgs rclcpp_action)

# Install the executable
install(TARGETS Utest_single_goal_distribution DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# If building tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)

  # Linter configuration
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


