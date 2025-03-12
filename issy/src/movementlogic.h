#ifndef MOVEMENTLOGIC_HPP
#define MOVEMENTLOGIC_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

class MovementLogic : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    MovementLogic();

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    double x_home, y_home;
    double x_goal, y_goal;
    bool move_now;

    void listen_for_input();
    void navigate_to(double x, double y);
};

#endif // MOVEMENTLOGIC_HPP
