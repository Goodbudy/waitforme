#include "GoalManager.h"
#include <algorithm>

GoalManager::GoalManager() : Node("goal_manager_node") {
    std::vector<std::string> bot_names = {"bot1", "bot2", "bot3"};
    create_status_subscribers(bot_names);

    // Timer to assign random goals for testing
    timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        [this]() {
            if (priority_order_.empty()) {
                RCLCPP_INFO(this->get_logger(), "No available bots to assign a goal.");
                return;
            }

            geometry_msgs::msg::Pose newGoal;
            newGoal.position.x = rand() % 10;
            newGoal.position.y = rand() % 10;
            newGoal.position.z = 0.0;

            bool isUrgent = (rand() % 2 == 0);

            std::string assigned_bot = priority_order_.front();  // Get top-priority bot
            RCLCPP_INFO(this->get_logger(), "Assigned %s to %s goal at (%.1f, %.1f)",
                        assigned_bot.c_str(), isUrgent ? "URGENT" : "normal",
                        newGoal.position.x, newGoal.position.y);
                    });
}

void GoalManager::create_status_subscribers(const std::vector<std::string> &bot_names) {
    for (const auto &bot : bot_names) {
        bot_status_map_[bot] = {false, true, false};  // default status

        at_home_subs_[bot] = this->create_subscription<std_msgs::msg::Bool>(
            "/" + bot + "/at_home", 10,
            [this, bot](const std_msgs::msg::Bool::SharedPtr msg) {
                status_callback(msg, bot, "at_home");
            });

        delivering_subs_[bot] = this->create_subscription<std_msgs::msg::Bool>(
            "/" + bot + "/delivering", 10,
            [this, bot](const std_msgs::msg::Bool::SharedPtr msg) {
                status_callback(msg, bot, "delivering");
            });

        proximity_subs_[bot] = this->create_subscription<std_msgs::msg::Bool>(
            "/" + bot + "/in_proximity", 10,
            [this, bot](const std_msgs::msg::Bool::SharedPtr msg) {
                status_callback(msg, bot, "in_proximity");
            });
    }
}

void GoalManager::status_callback(const std_msgs::msg::Bool::SharedPtr msg, const std::string &bot_id, const std::string &type) {
    if (bot_status_map_.find(bot_id) == bot_status_map_.end()) return;

    if (type == "at_home") bot_status_map_[bot_id].at_home = msg->data;
    else if (type == "delivering") bot_status_map_[bot_id].delivering = msg->data;
    else if (type == "in_proximity") bot_status_map_[bot_id].in_proximity = msg->data;

    update_priority_order();
}

void GoalManager::update_priority_order() {
    std::vector<std::pair<std::string, BotStatus>> sorted_bots(bot_status_map_.begin(), bot_status_map_.end());

    std::sort(sorted_bots.begin(), sorted_bots.end(), [](const auto &a, const auto &b) {
        if (a.second.at_home != b.second.at_home) return a.second.at_home > b.second.at_home;
        if (a.second.in_proximity != b.second.in_proximity) return a.second.in_proximity > b.second.in_proximity;
        return a.first < b.first;
    });

    priority_order_.clear();
    for (const auto &entry : sorted_bots) {
        if (entry.second.delivering) {
            priority_order_.push_back(entry.first);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Updated bot priority:");
    for (const auto &bot : priority_order_) {
        RCLCPP_INFO(this->get_logger(), "  %s", bot.c_str());
    }
}