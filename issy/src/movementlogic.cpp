
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <queue>

class MovementLogic : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    MovementLogic() : Node("movementlogic"), x_home(-2.0), y_home(0.0), tolerance(0.2), executing_goal(false) {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Subscribe to Odometry Data
        odom_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/odom", 10, std::bind(&MovementLogic::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Waiting for user input...");
        std::thread(&MovementLogic::listen_for_input, this).detach();
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;
    double x_home, y_home;
    double tolerance;
    double current_x = 0.0, current_y = 0.0;
    std::queue<std::pair<double, double>> goal_queue;
    bool executing_goal;

    void listen_for_input() {
        while (rclcpp::ok()) {
            std::string input;
            std::getline(std::cin, input);

            if (input.find("goal ") == 0) {
                double x, y;
                sscanf(input.c_str(), "goal %lf %lf", &x, &y);
                goal_queue.push({x, y});
                RCLCPP_INFO(this->get_logger(), "Added goal: x=%.2f, y=%.2f", x, y);
            }
            else if (input == "movenow" && !executing_goal) {
                executing_goal = true;
                execute_next_goal();
            }
        }
    }

    void execute_next_goal() {
        if (!goal_queue.empty()) {
            auto [x_goal, y_goal] = goal_queue.front();
            goal_queue.pop();
            navigate_to(x_goal, y_goal, [this]() {
                RCLCPP_INFO(this->get_logger(), "Arrived at goal! Starting 15s countdown...");
                for (int i = 15; i > 0; --i) {
                    RCLCPP_INFO(this->get_logger(), "Waiting... %d seconds left", i);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                RCLCPP_INFO(this->get_logger(), "Countdown complete! Returning to home base...");
                navigate_to(x_home, y_home, [this]() {
                    RCLCPP_INFO(this->get_logger(), "Returned to home base.");
                    print_remaining_goals();
                    executing_goal = false;
                    if (!goal_queue.empty()) {
                        execute_next_goal(); // Automatically continue to the next goal
                    }
                });
            });
        } else {
            RCLCPP_INFO(this->get_logger(), "No destination.");
            executing_goal = false;
        }
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
                RCLCPP_INFO(this->get_logger(), "Navigation successful! Position: x=%.2f, y=%.2f", current_x, current_y);
                on_success();
            }
        };

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_x = msg->pose.position.x;
        current_y = msg->pose.position.y;
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