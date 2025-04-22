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
#include <queue>
#include "issy/srv/add_goal.hpp"
#include "issy/srv/execute_goals.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class MovementLogic : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    MovementLogic();

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;
    rclcpp::Service<issy::srv::AddGoal>::SharedPtr add_goal_service_;
    rclcpp::Service<issy::srv::ExecuteGoals>::SharedPtr execute_goals_service_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    double x_home, y_home;
    double tolerance;
    double current_x = 0.0, current_y = 0.0;
    std::queue<std::pair<double, double>> goal_queue;
    bool executing_goal;
    bool home_base_reached = false;
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t current_index_;
    bool path_received_;

    void handle_add_goal(const std::shared_ptr<issy::srv::AddGoal::Request> request,
                          std::shared_ptr<issy::srv::AddGoal::Response> response);

    void handle_execute_goals(const std::shared_ptr<issy::srv::ExecuteGoals::Request> request,
                               std::shared_ptr<issy::srv::ExecuteGoals::Response> response);

    void listen_for_input();
    void execute_next_goal();
    void navigate_to(double x, double y, std::function<void()> on_success);
    void navigate_to_home_base();
    void odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void print_remaining_goals();
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
};

#endif // MOVEMENTLOGIC_HPP
