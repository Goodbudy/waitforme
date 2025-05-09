#include "GoalManager.h"
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "issy/srv/add_goal.hpp"
#include "issy/srv/execute_goals.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <queue>

using namespace std::chrono_literals;

GoalManager::GoalManager()
: Node("manager") {
    add_goal_client_ = this->create_client<issy::srv::AddGoal>("add_goal");
    exec_goals_client_ = this->create_client<issy::srv::ExecuteGoals>("execute_goals");

    check_timer_ = this->create_wall_timer(
        1s, std::bind(&GoalManager::update, this)
    );
}

void GoalManager::queue_global_goal(const geometry_msgs::msg::PoseStamped& goal_pose) {
    global_goal_queue_.push(goal_pose);
    RCLCPP_INFO(this->get_logger(), "[Manager] Queued global goal (%.2f, %.2f)",
                goal_pose.pose.position.x, goal_pose.pose.position.y);

    auto request = std::make_shared<issy::srv::AddGoal::Request>();
    request->x = goal_pose.pose.position.x;
    request->y = goal_pose.pose.position.y;

    if (!add_goal_client_->wait_for_service(2s)) {
        RCLCPP_ERROR(this->get_logger(), "[Manager] add_goal service not available");
        return;
    }

    add_goal_client_->async_send_request(request,
        [this](rclcpp::Client<issy::srv::AddGoal>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "[Manager] Goal added: %s", response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "[Manager] Goal add failed: %s", response->message.c_str());
            }
        });
}

void GoalManager::update() {
    if (!has_global_goals()) return;

    if (!exec_goals_client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(), "[Manager] execute_goals service not available");
        return;
    }

    auto request = std::make_shared<issy::srv::ExecuteGoals::Request>();
    exec_goals_client_->async_send_request(request,
        [this](rclcpp::Client<issy::srv::ExecuteGoals>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "[Manager] %s", response->message.c_str());
                global_goal_queue_.pop();
            } else {
                RCLCPP_INFO(this->get_logger(), "[Manager] No robots available to assign goal.");
            }
        });
}

bool GoalManager::has_global_goals() const {
    return !global_goal_queue_.empty();
}
