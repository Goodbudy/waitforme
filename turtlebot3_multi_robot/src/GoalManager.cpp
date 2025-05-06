// #include "GoalManager.h"

// #include <memory>
// #include <string>
// #include <chrono>
// #include <functional>
// #include <nav2_msgs/action/navigate_to_pose.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>

// using namespace std::chrono_literals;
// using NavigateToPose = nav2_msgs::action::NavigateToPose;
// using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// GoalManager::GoalManager()
// : Node("manager") {}

// void GoalManager::send_goal(const std::string& robot_namespace, const geometry_msgs::msg::PoseStamped& goal_pose) {
//     auto action_client = rclcpp_action::create_client<NavigateToPose>(
//         this,
//         "/" + robot_namespace + "/navigate_to_pose"
//     );

//     RCLCPP_INFO(this->get_logger(), "[%s] Sending goal...", robot_namespace.c_str());

//     if (!action_client->wait_for_action_server(10s)) {
//         RCLCPP_ERROR(this->get_logger(), "Action server not available for %s", robot_namespace.c_str());
//         return;
//     }

//     NavigateToPose::Goal goal;
//     goal.pose = goal_pose;

//     auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

//     send_goal_options.goal_response_callback =
//         [this, robot_namespace](GoalHandleNavigate::SharedPtr goal_handle) {
//             if (!goal_handle) {
//                 RCLCPP_ERROR(this->get_logger(), "[%s] Goal rejected by server", robot_namespace.c_str());
//             } else {
//                 RCLCPP_INFO(this->get_logger(), "[%s] Goal accepted", robot_namespace.c_str());
//             }
//         };

//     send_goal_options.result_callback =
//         [this, robot_namespace](const GoalHandleNavigate::WrappedResult & result) {
//             switch (result.code) {
//                 case rclcpp_action::ResultCode::SUCCEEDED:
//                     RCLCPP_INFO(this->get_logger(), "[%s] Goal succeeded!", robot_namespace.c_str());
//                     break;
//                 case rclcpp_action::ResultCode::ABORTED:
//                     RCLCPP_WARN(this->get_logger(), "[%s] Goal aborted", robot_namespace.c_str());
//                     break;
//                 case rclcpp_action::ResultCode::CANCELED:
//                     RCLCPP_WARN(this->get_logger(), "[%s] Goal canceled", robot_namespace.c_str());
//                     break;
//                 default:
//                     RCLCPP_ERROR(this->get_logger(), "[%s] Unknown result code", robot_namespace.c_str());
//                     break;
//             }
//         };

//     action_client->async_send_goal(goal, send_goal_options);
// }

// void GoalManager::register_bot(const std::string& name, std::shared_ptr<TurtleBot> bot) {
//     bots_[name] = bot;
//     if (pending_goals_.find(name) == pending_goals_.end()) {
//         pending_goals_[name] = std::queue<geometry_msgs::msg::PoseStamped>();
//         RCLCPP_INFO(this->get_logger(), "Registered TurtleBot: %s", name.c_str());
//     }    
// }

// void GoalManager::queue_goal(const std::string& name, const geometry_msgs::msg::PoseStamped& goal_pose) {
//     if (pending_goals_.find(name) != pending_goals_.end()) {
//         pending_goals_[name].push(goal_pose);
//         RCLCPP_INFO(this->get_logger(), "[%s] Queued new goal.", name.c_str());
//     } else {
//         RCLCPP_WARN(this->get_logger(), "[%s] Cannot queue goal: bot not registered.", name.c_str());
//     }
// }

// void GoalManager::update() {
//     for (auto& pair : pending_goals_) {
//         const std::string& name = pair.first;
//         auto& goal_queue = pair.second;
//         auto bot = bots_[name];

//         if (bot->isActionServerReady() && bot->isHome() && !goal_queue.empty()) {
//             const auto& next_goal = goal_queue.front();  // Peek without popping
//             bot->navigateTo(next_goal.pose.position.x, next_goal.pose.position.y);
//             bot->setAtHome(false);
//             goal_queue.pop();  // Pop only after the goal is dispatched
//             RCLCPP_INFO(this->get_logger(), "[%s] Assigned queued goal.", name.c_str());
//         }
//     }
// }

// bool GoalManager::has_pending_goals(const std::string& name) const {
//     auto it = pending_goals_.find(name);
//     if (it != pending_goals_.end()) {
//         return !it->second.empty();
//     }
//     return false;
// }

#include "GoalManager.h"
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

GoalManager::GoalManager()
: Node("manager") {}

void GoalManager::send_goal(const std::string& robot_namespace, const geometry_msgs::msg::PoseStamped& goal_pose) {
    auto action_client = rclcpp_action::create_client<NavigateToPose>(
        this,
        "/" + robot_namespace + "/navigate_to_pose"
    );

    RCLCPP_INFO(this->get_logger(), "[%s] Sending goal...", robot_namespace.c_str());

    if (!action_client->wait_for_action_server(10s)) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available for %s", robot_namespace.c_str());
        return;
    }

    NavigateToPose::Goal goal;
    goal.pose = goal_pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this, robot_namespace](GoalHandleNavigate::SharedPtr goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "[%s] Goal rejected by server", robot_namespace.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "[%s] Goal accepted", robot_namespace.c_str());
            }
        };

    send_goal_options.result_callback =
        [this, robot_namespace](const GoalHandleNavigate::WrappedResult & result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "[%s] Goal succeeded!", robot_namespace.c_str());
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(this->get_logger(), "[%s] Goal aborted", robot_namespace.c_str());
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "[%s] Goal canceled", robot_namespace.c_str());
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "[%s] Unknown result code", robot_namespace.c_str());
                    break;
            }
        };

    action_client->async_send_goal(goal, send_goal_options);
}

void GoalManager::register_bot(const std::string& name, std::shared_ptr<TurtleBot> bot) {
    if (bots_.find(name) == bots_.end()) {
        bots_[name] = bot;
        RCLCPP_INFO(this->get_logger(), "Registered TurtleBot: %s", name.c_str());
    }
}

void GoalManager::queue_global_goal(const geometry_msgs::msg::PoseStamped& goal_pose) {
    global_goal_queue_.push(goal_pose);
    RCLCPP_INFO(this->get_logger(), "[Manager] Queued global goal (%.2f, %.2f)", goal_pose.pose.position.x, goal_pose.pose.position.y);
}

bool GoalManager::has_global_goals() const {
    return !global_goal_queue_.empty();
}

void GoalManager::update() {
    for (auto& pair : bots_) {
        const std::string& name = pair.first;
        auto& bot = pair.second;

        if (bot->isActionServerReady() && bot->isHome() && has_global_goals()) {
            auto next_goal = global_goal_queue_.front();
            global_goal_queue_.pop();
            bot->navigateTo(next_goal.pose.position.x, next_goal.pose.position.y);
            bot->setAtHome(false);
            RCLCPP_INFO(this->get_logger(), "[%s] Assigned global queued goal.", name.c_str());
        }
    }
}

bool GoalManager::has_pending_goals(const std::string& name) const {
    // Deprecated: Using global queue now
    return has_global_goals();
}
