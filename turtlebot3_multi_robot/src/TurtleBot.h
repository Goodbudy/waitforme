#ifndef TURTLEBOT_H
#define TURTLEBOT_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <memory>

class TurtleBot {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    TurtleBot(std::string name, std::shared_ptr<rclcpp::Node> node);

    void receiveGoal(const geometry_msgs::msg::Pose& goal);

    void publishStatus();
    void setAtHome(bool);
    void setDelivering(bool);
    void setInProximity(bool);

    std::string getName() const;

    void navigateTo(double x, double y);
    void setHomePosition(double x, double y);
    void setGoalPosition(double x, double y);

    bool isActionServerReady();

private:
    std::string bot_name;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr at_home_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr delivering_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr proximity_pub_;

    bool is_home_;
    bool delivering_drink_;
    bool in_proximity_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    double x_home_, y_home_;
    double x_goal_, y_goal_;
};

#endif // TURTLEBOT_H

