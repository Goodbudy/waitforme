#include "path_planner.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

class PathPlanner : public rclcpp::Node {
    public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    PathPlanner() : Node("pathplanner") {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Wait for the action server to become available
        while (!client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        }

        // Start listening for input (goal commands or "movenow" command)
        std::thread(&PathPlanner::listen_for_input, this).detach();
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

    void listen_for_input() {
        while (rclcpp::ok()) {
            std::string input;
            std::getline(std::cin, input);

            if (input.find("goal ") == 0) {
                double x, y;
                sscanf(input.c_str(), "goal %lf %lf", &x, &y);
                //goal_queue.push({x, y});
                auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0; // Facing forward

        RCLCPP_INFO(this->get_logger(), "Sending goal...");
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        // Fix this

        //send_goal_options.result_callback = std::bind(&Nav2GoalSender::goal_result_callback, this, std::placeholders::_1);

        //this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }
            }
        
        }
    };

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}