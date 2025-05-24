#include "GoalManager.h"
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "issy/srv/add_goal.hpp"
#include "issy/srv/execute_goals.hpp"
#include "issy/srv/notify_idle.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <queue>

using namespace std::chrono_literals;

GoalManager::GoalManager()
: Node("manager") {
    add_goal_srv_ = this->create_service<issy::srv::AddGoal>(
    "/add_goal",
    std::bind(&GoalManager::handleAddGoal, this, std::placeholders::_1, std::placeholders::_2)
    );
    notify_idle_srv_ = this->create_service<issy::srv::NotifyIdle>(
        "/notify_idle",
        [this](const std::shared_ptr<issy::srv::NotifyIdle::Request> req,
               std::shared_ptr<issy::srv::NotifyIdle::Response> res) {
          robot_busy_[req->robot_namespace] = false;
          res->success = true;
          res->message = "Robot marked as idle.";
          RCLCPP_INFO(this->get_logger(), "[Manager] %s is now idle", req->robot_namespace.c_str());
        });
      
    exec_goals_client_["tb1"] = this->create_client<issy::srv::ExecuteGoals>("/tb1/execute_goals");
    exec_goals_client_["tb2"] = this->create_client<issy::srv::ExecuteGoals>("/tb2/execute_goals");

    robot_busy_["tb1"] = true;
    robot_busy_["tb2"] = true;

    
    // for (const auto& ns : robot_namespaces_) {
    //    exec_goals_client_[ns] = this->create_client<issy::srv::ExecuteGoals>("/" + ns + "/execute_goals");
    // }
    
    check_timer_ = this->create_wall_timer(
        1s, std::bind(&GoalManager::update, this)
    );
}

// Called externally via the add_goal service
void GoalManager::handleAddGoal(const std::shared_ptr<issy::srv::AddGoal::Request> req,
    std::shared_ptr<issy::srv::AddGoal::Response> res) {
geometry_msgs::msg::PoseStamped goal;
goal.header.frame_id = "map";
goal.header.stamp = now();
goal.pose.position.x = req->x;
goal.pose.position.y = req->y;
goal.pose.orientation.w = 1.0;

global_goal_queue_.push(goal);
res->success = true;
res->message = "Goal successfully added to global queue.";

RCLCPP_INFO(this->get_logger(), "[Manager] Goal received via service: (%.2f, %.2f)",
req->x, req->y);

std::queue<std::pair<double,double>> goal_queue_;
goal_queue_.emplace(req->x, req->y);

RCLCPP_INFO(this->get_logger(), "Added goal (%.2f, %.2f), queue size=%zu",
req->x, req->y, goal_queue_.size());
    }

// Optional: also allow internal tests to push goals
void GoalManager::queue_global_goal(const geometry_msgs::msg::PoseStamped& goal_pose) {
global_goal_queue_.push(goal_pose);
RCLCPP_INFO(this->get_logger(), "[Manager] Queued global goal (%.2f, %.2f)",
goal_pose.pose.position.x, goal_pose.pose.position.y);
}

void GoalManager::update() {

  for (const auto& [ns, busy] : robot_busy_) {
    RCLCPP_INFO(this->get_logger(), "[Manager] Robot '%s' busy state: %s", ns.c_str(), busy ? "true" : "false");
}

    if (!has_global_goals()) return;
  
    const auto& next_goal = global_goal_queue_.front();
  
    // Construct the goal execution request using the front of the queue
    auto request = std::make_shared<issy::srv::ExecuteGoals::Request>();
    request->x = next_goal.pose.position.x;
    request->y = next_goal.pose.position.y;
  
    bool goal_dispatched = false;
  
    for (auto& [ns, client] : exec_goals_client_) {
        if (robot_busy_[ns]) continue;
      
        if (!client->wait_for_service(500ms)) {
          RCLCPP_WARN(this->get_logger(), "[Manager] [%s] execute_goals service not available", ns.c_str());
          continue;
        }
      
        client->async_send_request(request,
          [this, ns](rclcpp::Client<issy::srv::ExecuteGoals>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
              RCLCPP_INFO(this->get_logger(), "[Manager] [%s] %s", ns.c_str(), response->message.c_str());
              global_goal_queue_.pop();
              robot_busy_[ns] = true;  // mark as busy
            } else {
              RCLCPP_INFO(this->get_logger(), "[Manager] [%s] Goal execution declined.", ns.c_str());
            }
          });
      
        goal_dispatched = true;
        break;  // Only assign one per cycle
      }
      
  
    if (!goal_dispatched) {
      RCLCPP_WARN(this->get_logger(), "[Manager] No available robots to execute goals.");
    }
  }
  

bool GoalManager::has_global_goals() const {
return !global_goal_queue_.empty();
}