#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "issy/srv/add_goal.hpp"
#include "issy/srv/execute_goals.hpp"
#include <queue>
#include <vector>
#include <functional>
#include <chrono>
#include <cmath>

enum class State { HOMING, IDLE, EXEC_GOAL_PATH, WAITING_AT_GOAL, RETURNING_HOME_PATH };

class MovementLogic : public rclcpp::Node {
public:
    MovementLogic()
        : Node("movementlogic"),
          x_home_(3.5),
          y_home_(3.2),
          tolerance_(0.2),
          state_(State::HOMING),
          current_target_idx_(0)
    {
        // Publishers
        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("astar_goal", 10);
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Subscriptions
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "planned_path", 10, std::bind(&MovementLogic::pathCallback, this, std::placeholders::_1));
        odom_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "odom", 10, std::bind(&MovementLogic::odomCallback, this, std::placeholders::_1));

        // Services
        add_goal_srv_ = create_service<issy::srv::AddGoal>(
            "add_goal", std::bind(&MovementLogic::handleAddGoal, this, std::placeholders::_1, std::placeholders::_2));
        exec_goals_srv_ = create_service<issy::srv::ExecuteGoals>(
            "execute_goals", std::bind(&MovementLogic::handleExecuteGoals, this, std::placeholders::_1, std::placeholders::_2));

        // Timer for path following
        follow_timer_ = create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&MovementLogic::followPath, this));

        // Start by homing
        RCLCPP_INFO(get_logger(), "Startup: homing to (%.2f, %.2f)", x_home_, y_home_);
        publishGoal(x_home_, y_home_);
    }

private:
    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;
    rclcpp::Service<issy::srv::AddGoal>::SharedPtr add_goal_srv_;
    rclcpp::Service<issy::srv::ExecuteGoals>::SharedPtr exec_goals_srv_;
    rclcpp::TimerBase::SharedPtr follow_timer_;
    rclcpp::TimerBase::SharedPtr wait_timer_;

    // State
    double x_home_, y_home_, tolerance_;
    State state_;
    std::queue<std::pair<double, double>> goal_queue_;
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t current_target_idx_;
    double current_x_{0.0}, current_y_{0.0};

    void publishGoal(double x, double y) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "map";
        msg.header.stamp = now();
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.orientation.w = 1.0;
        goal_pub_->publish(msg);
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty path");
            return;
        }
        path_ = msg->poses;
        current_target_idx_ = 0;
        RCLCPP_INFO(get_logger(), "Received path with %zu points", path_.size());
    }

    void odomCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_x_ = msg->pose.position.x;
        current_y_ = msg->pose.position.y;
    }

    void followPath() {
        if (current_target_idx_ >= path_.size()) return;
        auto &target = path_[current_target_idx_].pose;
        double dx = target.position.x - current_x_;
        double dy = target.position.y - current_y_;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < tolerance_) {
            current_target_idx_++;
            if (current_target_idx_ >= path_.size()) {
                RCLCPP_INFO(get_logger(), "Path complete in state=%d", static_cast<int>(state_));
                onPathComplete();
            }
            return;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.5 * dist;
        cmd.angular.z = std::atan2(dy, dx);
        cmd_vel_pub_->publish(cmd);
    }

    void onPathComplete() {
        switch (state_) {
            case State::HOMING:
                RCLCPP_INFO(get_logger(), "Home base reached");
                state_ = State::IDLE;
                break;
            case State::EXEC_GOAL_PATH:
                RCLCPP_INFO(get_logger(), "Goal reached, waiting 5s");
                state_ = State::WAITING_AT_GOAL;
                wait_timer_ = create_wall_timer(
                    std::chrono::seconds(5), std::bind(&MovementLogic::onWaitComplete, this));
                break;
            case State::RETURNING_HOME_PATH:
                RCLCPP_INFO(get_logger(), "Returned home from goal");
                if (!goal_queue_.empty()) {
                    state_ = State::EXEC_GOAL_PATH;
                    auto [x,y] = goal_queue_.front();
                    goal_queue_.pop();
                    publishGoal(x,y);
                    RCLCPP_INFO(get_logger(), "Next goal published: (%.2f, %.2f)", x, y);
                } else {
                    state_ = State::IDLE;
                    RCLCPP_INFO(get_logger(), "All goals complete, idle");
                }
                break;
            default:
                break;
        }
    }

    void onWaitComplete() {
        RCLCPP_INFO(get_logger(), "Wait complete, returning home");
        state_ = State::RETURNING_HOME_PATH;
        publishGoal(x_home_, y_home_);
    }

    void handleAddGoal(const std::shared_ptr<issy::srv::AddGoal::Request> req,
                       std::shared_ptr<issy::srv::AddGoal::Response> res) {
        goal_queue_.emplace(req->x, req->y);
        res->success = true;
        res->message = "Goal added";
        RCLCPP_INFO(get_logger(), "Added goal: (%.2f, %.2f)", req->x, req->y);
    }

    void handleExecuteGoals(const std::shared_ptr<issy::srv::ExecuteGoals::Request>,
                            std::shared_ptr<issy::srv::ExecuteGoals::Response> res) {
        if (state_ == State::IDLE && !goal_queue_.empty()) {
            state_ = State::EXEC_GOAL_PATH;
            auto [x,y] = goal_queue_.front();
            goal_queue_.pop();
            publishGoal(x,y);
            res->success = true;
            res->message = "Executing goal";
            RCLCPP_INFO(get_logger(), "Executing goal: (%.2f, %.2f)", x, y);
        } else {
            res->success = false;
            res->message = "Cannot execute goals in current state";
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementLogic>());
    rclcpp::shutdown();
    return 0;
}




// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include <iostream>
// #include <string>
// #include <thread>
// #include <chrono>
// #include <cmath>
// #include <queue>
// #include "issy/srv/add_goal.hpp"
// #include "issy/srv/execute_goals.hpp"

// using std::placeholders::_1;
// using std::placeholders::_2;

// class MovementLogic : public rclcpp::Node {
// public:
//     using NavigateToPose = nav2_msgs::action::NavigateToPose;
//     using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

//     MovementLogic() : Node("movementlogic"), x_home(3.5), y_home(3.2), tolerance(0.2), executing_goal(false) {
//         client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

//         // Wait for the action server to become available
//         while (!client_->wait_for_action_server(std::chrono::seconds(1))) {
//             RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
//         }

//         // Subscribe to Odometry Data
//         odom_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//             "/odom", 10, std::bind(&MovementLogic::odom_callback, this, std::placeholders::_1));

//         // Initiate movement to home base as soon as the node starts
//         RCLCPP_INFO(this->get_logger(), "Navigating to home base...");
//         navigate_to_home_base();

//         // Start the services
//         add_goal_service_ = this->create_service<issy::srv::AddGoal>(
//             "add_goal", std::bind(&MovementLogic::handle_add_goal, this, _1, _2));

//         execute_goals_service_ = this->create_service<issy::srv::ExecuteGoals>(
//             "execute_goals", std::bind(&MovementLogic::handle_execute_goals, this, _1, _2));   

//     }

// private:
//     rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;
//     rclcpp::Service<issy::srv::AddGoal>::SharedPtr add_goal_service_;
//     rclcpp::Service<issy::srv::ExecuteGoals>::SharedPtr execute_goals_service_;
    
//     double x_home, y_home;
//     double tolerance;
//     double current_x = 0.0, current_y = 0.0;
//     std::queue<std::pair<double, double>> goal_queue;
//     bool executing_goal;
//     bool home_base_reached = false;

//     void handle_add_goal(const std::shared_ptr<issy::srv::AddGoal::Request> request,
//         std::shared_ptr<issy::srv::AddGoal::Response> response) {
//         goal_queue.push({request->x, request->y});
//         response->success = true;
//         response->message = "Goal added successfully.";
//         RCLCPP_INFO(this->get_logger(), "Service: Added goal x=%.2f, y=%.2f", request->x, request->y);
//     }

//     void handle_execute_goals(const std::shared_ptr<issy::srv::ExecuteGoals::Request> /*request*/,
//              std::shared_ptr<issy::srv::ExecuteGoals::Response> response) {
//         if (!executing_goal && home_base_reached) {
//             executing_goal = true;
//             execute_next_goal();
//             response->success = true;
//             response->message = "Executing next goal.";
//         } else {
//             response->success = false;
//             response->message = "Already executing or home base not yet reached.";
//         }
//     }

//     void execute_next_goal() {
//         if (!goal_queue.empty()) {
//             auto [x_goal, y_goal] = goal_queue.front();
//             goal_queue.pop();
//             navigate_to(x_goal, y_goal, [this]() {
//                 RCLCPP_INFO(this->get_logger(), "Arrived at goal! Starting 5s countdown...");
//                 for (int i = 5; i > 0; --i) {
//                     RCLCPP_INFO(this->get_logger(), "Waiting... %d seconds left", i);
//                     std::this_thread::sleep_for(std::chrono::seconds(1));
//                 }
//                 RCLCPP_INFO(this->get_logger(), "Countdown complete! Returning to home base...");
//                 navigate_to_home_base();
//             });
//         } else {
//             RCLCPP_INFO(this->get_logger(), "No goals in the queue.");
//             executing_goal = false;
//         }
//     }

//     void navigate_to(double x, double y, std::function<void()> on_success) {
//         auto goal_msg = NavigateToPose::Goal();
//         goal_msg.pose.header.frame_id = "map";
//         goal_msg.pose.header.stamp = this->now();
//         goal_msg.pose.pose.position.x = x;
//         goal_msg.pose.pose.position.y = y;
//         goal_msg.pose.pose.orientation.w = 1.0;

//         auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//         send_goal_options.result_callback = [this, on_success](const GoalHandleNav::WrappedResult & result) {
//             if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
//                 RCLCPP_INFO(this->get_logger(), "Navigation successful!");
//                 on_success();
//             }
//         };

//         client_->async_send_goal(goal_msg, send_goal_options);
//     }

//     void navigate_to_home_base() {
//         navigate_to(x_home, y_home, [this]() {
//             RCLCPP_INFO(this->get_logger(), "Returned to home base.");
//             home_base_reached = true; // Mark home base as reached
//             executing_goal = false;
//             print_remaining_goals();
//         });
//     }

//     void odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//         current_x = msg->pose.position.x;
//         current_y = msg->pose.position.y;
//     }

//     void print_remaining_goals() {
//         if (goal_queue.empty()) {
//             RCLCPP_INFO(this->get_logger(), "Goal list is empty.");
//         } else {
//             RCLCPP_INFO(this->get_logger(), "Remaining goals:");
//             std::queue<std::pair<double, double>> temp_queue = goal_queue;
//             while (!temp_queue.empty()) {
//                 auto [x, y] = temp_queue.front();
//                 temp_queue.pop();
//                 RCLCPP_INFO(this->get_logger(), " - x=%.2f, y=%.2f", x, y);
//             }
//         }
//     }
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MovementLogic>());
//     rclcpp::shutdown();
//     return 0;
// }
