#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <thread>
#include <string>

class Localisation : public rclcpp::Node {
public:
    Localisation();

private:
    void user_input();  // Thread function
    void set_pose(double x, double y);  // Pose setter

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    std::thread input_thread_;
};
