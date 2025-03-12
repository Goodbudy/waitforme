#include <rclcpp.hpp>

int main (in argc, char **argv){
    rclcpp::init(argc,argv);
    RCLCPP_INFO(rclcpp::get_logger("hello world"), "hello worls");
    rclcpp::shutdown();
    return 0;
}