#include "TurtleBot.h"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

TurtleBot::TurtleBot(std::string name, std::shared_ptr<rclcpp::Node> node)
    : bot_name(name), node_(node), 
    at_home_pub_(nullptr), delivering_pub_(nullptr), proximity_pub_(nullptr),
    is_home_(true), delivering_drink_(false), in_proximity_(false),
    goal_active_(false), goal_tolerance_(0.2),
    x_home_(0.0), y_home_(0.0), x_goal_(0.0), y_goal_(0.0),
    returning_home_(false)

{
    at_home_pub_ = node_->create_publisher<std_msgs::msg::Bool>(bot_name + "/at_home", 10);
    delivering_pub_ = node_->create_publisher<std_msgs::msg::Bool>(bot_name + "/delivering", 10);
    proximity_pub_ = node_->create_publisher<std_msgs::msg::Bool>(bot_name + "/proximity", 10);

    client_ = rclcpp_action::create_client<NavigateToPose>(
        node_, "/" + bot_name + "/navigate_to_pose");

    send_goal_options_.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
        RCLCPP_INFO(node_->get_logger(), "[%s] Navigation result received", bot_name.c_str());
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                if (!returning_home_) {
                    RCLCPP_INFO(node_->get_logger(), "[%s] Delivery complete. Returning home...", bot_name.c_str());
                    returning_home_ = true;
                    navigateTo(x_home_, y_home_);
                } else {
                    RCLCPP_INFO(node_->get_logger(), "[%s] Arrived home. Ready for next goal.", bot_name.c_str());
                    setAtHome(true);
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(), "[%s] Navigation aborted", bot_name.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(node_->get_logger(), "[%s] Navigation cancelled", bot_name.c_str());
                break;
            default:
                RCLCPP_WARN(node_->get_logger(), "[%s] Unknown result code", bot_name.c_str());
                break;
        }
    };

    RCLCPP_INFO(node_->get_logger(), "[%s] TurtleBot initialized", bot_name.c_str());
}

bool TurtleBot::isHome() const {
    return is_home_;
}

bool TurtleBot::isActionServerReady() {
    return client_->wait_for_action_server(std::chrono::seconds(1));
}

void TurtleBot::publishStatus() {
    std_msgs::msg::Bool msg;

    msg.data = is_home_;
    at_home_pub_->publish(msg);

    msg.data = delivering_drink_;
    delivering_pub_->publish(msg);

    msg.data = in_proximity_;
    proximity_pub_->publish(msg);
}

void TurtleBot::setAtHome(bool at_home) {
    is_home_ = at_home;
}

void TurtleBot::setDelivering(bool delivering) {
    delivering_drink_ = delivering;
}

void TurtleBot::setInProximity(bool in_prox) {
    in_proximity_ = in_prox;
}

std::string TurtleBot::getName() const {
    return bot_name;
}

void TurtleBot::navigateTo(double x, double y) {
    returning_home_ = (x == x_home_ && y == y_home_);
    is_home_ = false;

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(node_->get_logger(), "[%s] Sending goal to (%.2f, %.2f)...", bot_name.c_str(), x, y);

    client_->async_send_goal(goal_msg, send_goal_options_);
}

void TurtleBot::setHomePosition(double x, double y) {
    x_home_ = x;
    y_home_ = y;
}

void TurtleBot::setGoalPosition(double x, double y) {
    x_goal_ = x;
    y_goal_ = y;
}
