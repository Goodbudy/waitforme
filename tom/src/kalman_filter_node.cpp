#include "kalman_filter_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <eigen3/Eigen/Dense>

KalmanFilterNode::KalmanFilterNode() : Node("kalman_filter_node"), is_initialized_(false)
{
    // Initialize state and covariance
    x_ = Eigen::VectorXd::Zero(6);
    P_ = Eigen::MatrixXd::Identity(6, 6);

    // Define state transition model (6x6 matrix)
    F_ = Eigen::MatrixXd::Identity(6, 6);

    // Define process noise covariance (6x6 matrix)
    Q_ = 0.01 * Eigen::MatrixXd::Identity(6, 6);

    // Define measurement model (3x6 matrix)
    H_ = Eigen::MatrixXd::Zero(3, 6);
    H_(0, 0) = 1; // x position
    H_(1, 1) = 1; // y position
    H_(2, 2) = 1; // theta orientation

    // Define measurement noise covariance (3x3 matrix)
    R_ = 0.05 * Eigen::MatrixXd::Identity(3, 3);

    // Subscribe to odometry data
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&KalmanFilterNode::odomCallback, this, std::placeholders::_1));
}

void KalmanFilterNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    Eigen::VectorXd z(3);
    z << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z;

    if (!is_initialized_)
    {
        x_(0) = z(0);
        x_(1) = z(1);
        x_(2) = z(2);
        is_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Kalman Filter Initialized at x=%.2f, y=%.2f, theta=%.2f", x_(0), x_(1), x_(2));
        return;
    }

    // Predict and Update
    predict();
    update(z);

    RCLCPP_INFO(this->get_logger(), "State: x=%.2f, y=%.2f, theta=%.2f", x_(0), x_(1), x_(2));
}

void KalmanFilterNode::predict()
{
    // State prediction
    x_ = F_ * x_;

    // Covariance prediction
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilterNode::update(const Eigen::VectorXd &z)
{
    Eigen::VectorXd y = z - H_ * x_;                // Measurement residual
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_; // Residual covariance
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse(); // Kalman gain

    // Update state and covariance
    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * P_;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
