#include "TurtleBot.h"

TurtleBot::TurtleBot(std::string name, std::shared_ptr<rclcpp::Node> node)
: bot_name(name), node_(node), is_home_(false), delivering_drink_(false), in_proximity_(false)
{
    at_home_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/" + bot_name + "/at_home", 10);
    delivering_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/" + bot_name + "/delivering", 10);
    proximity_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/" + bot_name + "/in_proximity", 10);
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

void TurtleBot::setAtHome(bool status) {
    is_home_ = status;
}

void TurtleBot::setDelivering(bool status) {
    delivering_drink_ = status;
}

void TurtleBot::setInProximity(bool status) {
    in_proximity_ = status;
}

std::string TurtleBot::getName() const {
    return bot_name;
}