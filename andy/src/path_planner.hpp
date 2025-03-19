#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class PathPlanner {
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    
        // Constructor
        PathPlanner(rclcpp::Node::SharedPtr node) : node_(node) {
            action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
        }
        

        // Function to send a goal to the action server
        // Only implimented to test path planning is working and proove through simulated robot movement. 
        // This will be redundent after controller for the robot is created, we will only send the path movement to their functions. 
    void send_goal(const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Nav2 action server not available!");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = *goal;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [](const GoalHandleNav::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(rclcpp::get_logger("GoalSender"), "Goal succeeded!");
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("GoalSender"), "Goal failed.");
            }
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

#endif  // PATH_PLANNER_HPP
