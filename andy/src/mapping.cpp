#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("hello_world"), "Hello, World!");
    rclcpp::shutdown();
    return 0;
}