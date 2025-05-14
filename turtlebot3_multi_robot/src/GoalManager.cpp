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
    add_goal_srv_ = this->create_service<issy::srv::AddGoal>(
    "add_goal",
    std::bind(&GoalManager::handleAddGoal, this, std::placeholders::_1, std::placeholders::_2)
    );
    exec_goals_clients_[tb1] = this->create_client<issy::srv::ExecuteGoals>("/tb1/execute_goals");
    exec_goals_clients_[tb2] = this->create_client<issy::srv::ExecuteGoals>("/tb2/execute_goals");
    
    // for (const auto& ns : robot_namespaces_) {
    //    exec_goals_clients_[ns] = this->create_client<issy::srv::ExecuteGoals>("/" + ns + "/execute_goals");
    // }
    
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

    auto request = std::make_shared<issy::srv::ExecuteGoals::Request>();
    bool goal_dispatched = false;

    for (auto& [ns, client] : exec_goals_clients_) {
        if (!client->wait_for_service(500ms)) {
            RCLCPP_WARN(this->get_logger(), "[Manager] [%s] execute_goals service not available", ns.c_str());
            continue;
        }

        // Capture namespace for use in callback
        client->async_send_request(request,
            [this, ns](rclcpp::Client<issy::srv::ExecuteGoals>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "[Manager] [%s] %s", ns.c_str(), response->message.c_str());
                    global_goal_queue_.pop();  // Only pop once someone has accepted
                } else {
                    RCLCPP_INFO(this->get_logger(), "[Manager] [%s] Goal execution declined.", ns.c_str());
                }
            });

        // Send request to only one robot per cycle
        goal_dispatched = true;
        break;
    }

    if (!goal_dispatched) {
        RCLCPP_WARN(this->get_logger(), "[Manager] No available robots to execute goals.");
    }
}

bool GoalManager::has_global_goals() const {
    return !global_goal_queue_.empty();
}
