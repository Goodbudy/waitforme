#ifndef KALMAN_FILTER_NODE_HPP
#define KALMAN_FILTER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <eigen3/Eigen/Dense>

class KalmanFilterNode : public rclcpp::Node
{
public:
    KalmanFilterNode();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void predict();
    void update(const Eigen::VectorXd &z);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    bool is_initialized_ = false; // Properly declared and initialized

    // Kalman Filter variables
    Eigen::VectorXd x_;  // State vector [x, y, theta, vx, vy, vtheta]
    Eigen::MatrixXd P_;  // Covariance matrix
    Eigen::MatrixXd F_;  // State transition model
    Eigen::MatrixXd Q_;  // Process noise covariance
    Eigen::MatrixXd H_;  // Measurement model
    Eigen::MatrixXd R_;  // Measurement noise covariance
};

#endif // KALMAN_FILTER_NODE_HPP
