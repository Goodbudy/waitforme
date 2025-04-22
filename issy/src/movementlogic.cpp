#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <queue>
#include "issy/srv/add_goal.hpp"
#include "issy/srv/execute_goals.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class MovementLogic : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    MovementLogic() : Node("movementlogic"), x_home(3.5), y_home(3.2), tolerance(0.2), executing_goal(false) {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Wait for the action server to become available
        while (!client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        }

        // Subscribe to Odometry Data
        odom_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/odom", 10, std::bind(&MovementLogic::odom_callback, this, std::placeholders::_1));

        // Subscribe to the path topic
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10, std::bind(&MovementLogic::path_callback, this, _1));

        // Publish velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/astar_goal", 10);

        // Initiate movement to home base as soon as the node starts
        RCLCPP_INFO(this->get_logger(), "Navigating to home base...");
        navigate_to_home_base();

        // Start the services
        add_goal_service_ = this->create_service<issy::srv::AddGoal>(
            "add_goal", std::bind(&MovementLogic::handle_add_goal, this, _1, _2));

        execute_goals_service_ = this->create_service<issy::srv::ExecuteGoals>(
            "execute_goals", std::bind(&MovementLogic::handle_execute_goals, this, _1, _2));   
        
        // Timer to execute path-following
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MovementLogic::follow_path, this));
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Service<issy::srv::AddGoal>::SharedPtr add_goal_service_;
    rclcpp::Service<issy::srv::ExecuteGoals>::SharedPtr execute_goals_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;

    double x_home, y_home;
    double tolerance;
    double current_x = 0.0, current_y = 0.0;
    std::queue<std::pair<double, double>> goal_queue;
    std::vector<geometry_msgs::msg::PoseStamped> current_path_;
    size_t current_target_index_ = 0;
    bool executing_goal;
    bool home_base_reached = false;
    bool following_path_ = false;

    void handle_add_goal(const std::shared_ptr<issy::srv::AddGoal::Request> request,
        std::shared_ptr<issy::srv::AddGoal::Response> response) {
        goal_queue.push({request->x, request->y});
        response->success = true;
        response->message = "Goal added successfully.";
        RCLCPP_INFO(this->get_logger(), "Service: Added goal x=%.2f, y=%.2f", request->x, request->y);
    }

    void handle_execute_goals(const std::shared_ptr<issy::srv::ExecuteGoals::Request> /*request*/,
             std::shared_ptr<issy::srv::ExecuteGoals::Response> response) {
        if (!executing_goal && home_base_reached) {
            executing_goal = true;
            execute_next_goal();
            response->success = true;
            response->message = "Executing next goal.";
        } else {
            response->success = false;
            response->message = "Already executing or home base not yet reached.";
        }
    }

    // void execute_next_goal() {
    //     if (!goal_queue.empty()) {
    //         auto [x_goal, y_goal] = goal_queue.front();
    //         goal_queue.pop();
    //         navigate_to(x_goal, y_goal, [this]() {
    //             RCLCPP_INFO(this->get_logger(), "Arrived at goal! Starting 5s countdown...");
    //             for (int i = 5; i > 0; --i) {
    //                 RCLCPP_INFO(this->get_logger(), "Waiting... %d seconds left", i);
    //                 std::this_thread::sleep_for(std::chrono::seconds(1));
    //             }
    //             RCLCPP_INFO(this->get_logger(), "Countdown complete! Returning to home base...");
    //             navigate_to_home_base();
    //         });
    //     } else {
    //         RCLCPP_INFO(this->get_logger(), "No goals in the queue.");
    //         executing_goal = false;
    //     }
    // }

    void execute_next_goal() {
        if (!goal_queue.empty()) {
            auto [x_goal, y_goal] = goal_queue.front();
            goal_queue.pop();
    
            publish_goal_to_astar(x_goal, y_goal);
    
            RCLCPP_INFO(this->get_logger(), "Published goal to astar planner: x=%.2f, y=%.2f", x_goal, y_goal);
        } else {
            RCLCPP_INFO(this->get_logger(), "No goals in the queue.");
            executing_goal = false;
        }
    }    

    void publish_goal_to_astar(double x, double y) {
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.frame_id = "map";
        goal_msg.header.stamp = this->now();
        goal_msg.pose.position.x = x;
        goal_msg.pose.position.y = y;
        goal_msg.pose.orientation.w = 1.0;
    
        goal_publisher_->publish(goal_msg);
    }    

    void navigate_to(double x, double y, std::function<void()> on_success) {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this, on_success](const GoalHandleNav::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Navigation successful!");
                on_success();
            }
        };

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void navigate_to_home_base() {
        navigate_to(x_home, y_home, [this]() {
            RCLCPP_INFO(this->get_logger(), "Returned to home base.");
            home_base_reached = true; // Mark home base as reached
            executing_goal = false;
            print_remaining_goals();
        });
    }

    void odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_x = msg->pose.position.x;
        current_y = msg->pose.position.y;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) return;
        current_path_ = msg->poses;
        current_target_index_ = 0;
        following_path_ = true;
        RCLCPP_INFO(this->get_logger(), "Received new path with %zu poses", current_path_.size());
    }

    void follow_path() {
        if (!following_path_ || current_target_index_ >= current_path_.size()) return;

        auto target_pose = current_path_[current_target_index_].pose;
        double dx = target_pose.position.x - current_x;
        double dy = target_pose.position.y - current_y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < 0.2) {
            current_target_index_++;
            if (current_target_index_ >= current_path_.size()) {
                RCLCPP_INFO(this->get_logger(), "Reached final path goal.");
                following_path_ = false;
            }
            return;
        }

        // Simple proportional control
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.5 * distance;
        cmd_vel.angular.z = 1.0 * std::atan2(dy, dx);
        cmd_vel_pub_->publish(cmd_vel);
    }

    void print_remaining_goals() {
        if (goal_queue.empty()) {
            RCLCPP_INFO(this->get_logger(), "Goal list is empty.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Remaining goals:");
            std::queue<std::pair<double, double>> temp_queue = goal_queue;
            while (!temp_queue.empty()) {
                auto [x, y] = temp_queue.front();
                temp_queue.pop();
                RCLCPP_INFO(this->get_logger(), " - x=%.2f, y=%.2f", x, y);
            }
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
