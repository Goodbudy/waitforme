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
add_executable(MainExe_Utest src/MainExe_Utest.cpp src/TurtleBot.cpp src/GoalManager.cpp)

# Link dependencies to your executable
ament_target_dependencies(MainExe_Utest rclcpp std_msgs geometry_msgs nav_msgs nav2_msgs rclcpp_action)

# Install the executable
install(TARGETS MainExe_Utest DESTINATION lib/${PROJECT_NAME})

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


