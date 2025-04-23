#include "localisation.h"
#include <iostream>
#include <chrono>
#include <sstream>

Localisation::Localisation() : Node("localisation_node") {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    RCLCPP_INFO(this->get_logger(), "Localisation Node Started");
    input_thread_ = std::thread(&Localisation::user_input, this);
    input_thread_.detach();  // let it run independently
}

void Localisation::user_input() {
    while (rclcpp::ok()) {
        std::string input;
        std::getline(std::cin, input);

        if (input.find("setpose ") == 0) {
            double x, y;
            sscanf(input.c_str(), "setpose %lf %lf", &x, &y);
            RCLCPP_INFO(this->get_logger(), "Setting pose to X: %.2f, Y: %.2f", x, y);
            set_pose(x, y);
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", input.c_str());
        }
    }
}


void Localisation::set_pose(double x, double y) {
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.w = 1.0;

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published initial pose at x=%.2f, y=%.2f", x, y);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Localisation>());
    rclcpp::shutdown();
    return 0;
}
