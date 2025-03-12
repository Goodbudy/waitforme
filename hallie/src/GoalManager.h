#ifndef GOAL_MANAGER_H
#define GOAL_MANAGER_H

#include <vector>
#include <deque>
#include <unordered_map>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>

struct BotStatus {
    bool at_home;
    bool delivering;
    bool in_proximity;
};

class GoalManager : public rclcpp::Node {
public:
    GoalManager();

private:
    std::unordered_map<std::string, BotStatus> bot_status_map_;
    std::vector<std::string> priority_order_;

    rclcpp::TimerBase::SharedPtr timer_;

    void update_priority_order();
    void create_status_subscribers(const std::vector<std::string> &bot_names);

    std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> at_home_subs_;
    std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> delivering_subs_;
    std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> proximity_subs_;

    void status_callback(const std_msgs::msg::Bool::SharedPtr msg, const std::string &bot_id, const std::string &type);
};

#endif
