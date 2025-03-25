#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

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

class PathPlanner : public rclcpp::Node {
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    
        PathPlanner();
    
    private:
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        double x_goal, y_goal;  
        double current_x;
        double current_y;
        void listen_for_input();
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    };
    
    #endif // PATHPLANNER_HPP
