#include "GoalManager.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalManager>());
  rclcpp::shutdown();
  return 0;
}
