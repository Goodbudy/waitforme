#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <queue>
#include <string>
#include <memory>
#include <map>

#include "issy/srv/add_goal.hpp"
#include "issy/srv/execute_goals.hpp"

class GoalManager : public rclcpp::Node {
public:
    GoalManager();

    void queue_global_goal(const geometry_msgs::msg::PoseStamped& goal_pose);  // Optional: for test code
    bool has_global_goals() const;
    void update();

private:
    std::queue<geometry_msgs::msg::PoseStamped> global_goal_queue_;

    rclcpp::Service<issy::srv::AddGoal>::SharedPtr add_goal_srv_;
    std::map<std::string, rclcpp::Client<issy::srv::ExecuteGoals>::SharedPtr> exec_goals_client_;
    rclcpp::TimerBase::SharedPtr check_timer_;

    void handleAddGoal(
        const std::shared_ptr<issy::srv::AddGoal::Request> req,
        std::shared_ptr<issy::srv::AddGoal::Response> res);
};
