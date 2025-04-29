#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <unordered_map>
#include <queue>
#include <memory>
#include <string>

#include "TurtleBot.h"  // <-- USE YOUR TurtleBot CLASS!

class GoalManager : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoalManager();

    void send_goal(const std::string& robot_namespace, const geometry_msgs::msg::PoseStamped& goal_pose);

    void register_bot(const std::string& name, std::shared_ptr<TurtleBot> bot);

    void queue_goal(const std::string& name, const geometry_msgs::msg::PoseStamped& goal_pose);

    void update();

    bool has_pending_goals(const std::string& name) const;

private:
    std::unordered_map<std::string, std::shared_ptr<TurtleBot>> bots_;
    std::unordered_map<std::string, std::queue<geometry_msgs::msg::PoseStamped>> pending_goals_;
};

