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

class MovementLogic : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    MovementLogic() : Node("movementlogic"), x_home(-2.0), y_home(0.0), tolerance(0.2) {
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
    double x_goal, y_goal;
    double tolerance;
    double current_x = 0.0, current_y = 0.0;  // Updated position variables

    void listen_for_input() {
        while (rclcpp::ok()) {
            std::string input;
            std::getline(std::cin, input);

            if (input.find("goal ") == 0) {
                sscanf(input.c_str(), "goal %lf %lf", &x_goal, &y_goal);
                RCLCPP_INFO(this->get_logger(), "Received goal: x=%.2f, y=%.2f", x_goal, y_goal);
                navigate_to(x_home, y_home);
                RCLCPP_INFO(this->get_logger(), "Travelling to home base");
            }
            else if (input == "movenow") {
                RCLCPP_INFO(this->get_logger(), "Received 'movenow' command. Navigating to goal...");
                navigate_to(x_goal, y_goal);
            }
        }
    }

    void navigate_to(double x, double y) {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = 
            [this, x, y](const GoalHandleNav::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Navigation successful!");

                    // Check if the goal was the user-defined goal
                    if (std::fabs(x - x_goal) <= tolerance && std::fabs(y - y_goal) <= tolerance) {
                        RCLCPP_INFO(this->get_logger(), "Arrived at goal! Starting 15s countdown...");
                        
                        // 15-second countdown
                        for (int i = 15; i > 0; --i) {
                            RCLCPP_INFO(this->get_logger(), "Waiting... %d seconds left", i);
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                        }

                        RCLCPP_INFO(this->get_logger(), "Countdown complete! Returning to home base...");
                        navigate_to(x_home, y_home);
                    } else if (std::fabs(x - x_home) <= tolerance && std::fabs(y - y_home) <= tolerance) {
                        RCLCPP_INFO(this->get_logger(), "Returned to home base.");
                    }
                }
            };

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_x = msg->pose.position.x;
        current_y = msg->pose.position.y;

        RCLCPP_INFO(this->get_logger(), "Current Position Updated: x=%.2f, y=%.2f", current_x, current_y);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementLogic>());
    rclcpp::shutdown();
    return 0;
}
