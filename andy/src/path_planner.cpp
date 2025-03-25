#include "path_planner.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

PathPlanner::PathPlanner() : Node("pathplanner")
{

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Wait for the action server to become available
    while (!client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }

    // Subscribe to Odometry Data
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PathPlanner::odom_callback, this, std::placeholders::_1));

    // Start listening for input (goal X Y or return to home)
    std::thread(&PathPlanner::listen_for_input, this).detach();
    RCLCPP_INFO(this->get_logger(), "Path Planner Node Activated");
}

void PathPlanner::listen_for_input()
{
    while (rclcpp::ok())
    {
        std::string input;
        std::getline(std::cin, input);

        if (input.find("goal ") == 0)
        {
            double x, y;
            sscanf(input.c_str(), "goal %lf %lf", &x, &y);
            auto goal_msg = NavigateToPose::Goal();
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.header.stamp = this->get_clock()->now();
            goal_msg.pose.pose.position.x = x;
            goal_msg.pose.pose.position.y = y;
            goal_msg.pose.pose.orientation.w = 1.0; // Facing forward

            // home is x -2.0 y -0.5

            RCLCPP_INFO(this->get_logger(), "Sending goal X: %.2f Y: %.2f", x, y);
            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

            this->client_->async_send_goal(goal_msg, send_goal_options);
        }
        if (input.find("return to home") == 0)
        {
            auto goal_msg = NavigateToPose::Goal();
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.header.stamp = this->get_clock()->now();
            goal_msg.pose.pose.position.x = -2.0;
            goal_msg.pose.pose.position.y = -0.5;
            goal_msg.pose.pose.orientation.w = 1.0; // Facing forward

            RCLCPP_INFO(this->get_logger(), "returning to home");
            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

            this->client_->async_send_goal(goal_msg, send_goal_options);
        }
        if (input.find("position") == 0)
        {
        double x = current_x;
        double y = current_y;
        RCLCPP_INFO(this->get_logger(), "Current position X: %.2f Y: %.2f", x, y);
        }
    }
}

void PathPlanner::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}