#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <string>

class TurtleBot {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    TurtleBot(std::string name, std::shared_ptr<rclcpp::Node> node);

    bool isActionServerReady();

    void publishStatus();

    void setAtHome(bool at_home);
    bool isHome() const;
    void setDelivering(bool delivering);
    void setInProximity(bool in_prox);

    std::string getName() const;

    void navigateTo(double x, double y);

    void setHomePosition(double x, double y);
    void setGoalPosition(double x, double y);

private:
    std::string bot_name;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr at_home_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr delivering_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr proximity_pub_;

    bool is_home_;
    bool delivering_drink_;
    bool in_proximity_;

    double x_home_;
    double y_home_;
    double x_goal_;
    double y_goal_;
};

