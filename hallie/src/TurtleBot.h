#ifndef TURTLEBOT_H
#define TURTLEBOT_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

class TurtleBot {
public:
    TurtleBot(std::string name, std::shared_ptr<rclcpp::Node> node);

    void publishStatus();
    void setAtHome(bool);
    void setDelivering(bool);
    void setInProximity(bool);

    std::string getName() const;

private:
    std::string bot_name;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr at_home_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr delivering_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr proximity_pub_;

    bool is_home_;
    bool delivering_drink_;
    bool in_proximity_;
};

#endif
