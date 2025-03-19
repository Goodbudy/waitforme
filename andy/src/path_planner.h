#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
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
        double x_goal, y_goal;
        void listen_for_input();
    };
    
    #endif // PATHPLANNER_HPP
