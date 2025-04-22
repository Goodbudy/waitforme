cmake_minimum_required(VERSION 3.5)
project(issy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(andy REQUIRED)

# Service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddGoal.srv"
  "srv/ExecuteGoals.srv"
)

add_executable(movementlogic src/movementlogic.cpp)
ament_target_dependencies(movementlogic
  rclcpp
  nav2_msgs
  rclcpp_action
  geometry_msgs
  std_msgs
  andy
)

# Link generated services
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(movementlogic ${cpp_typesupport_target})

install(TARGETS
  movementlogic
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
