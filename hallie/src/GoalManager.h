#ifndef GOAL_MANAGER_H
#define GOAL_MANAGER_H

#include <vector>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include "TurtleBot.h"  // Assuming you have a separate TurtleBot class header

class GoalManager {
private:
    std::vector<TurtleBot> turtleBots;  // List of available TurtleBots
    std::deque<geometry_msgs::PoseStamped> goalQueue;  // Queue for normal goals
    std::deque<geometry_msgs::PoseStamped> urgentGoalQueue;  // Queue for urgent goals
    ros::Publisher barRobotPub;  // Publisher for bar robot status
    ros::Subscriber botStatusSub;  // Subscriber for TurtleBot status updates

public:
    // Constructor for GoalManager class
    GoalManager(std::vector<TurtleBot> &bots, ros::NodeHandle &nh);

    // Method to add a goal to the queue
    void addGoalToQueue(const geometry_msgs::PoseStamped &goal, bool isUrgent = false);

    // Method to handle updates on TurtleBot status (whether they are home or not)
    void updateBotStatus(const std_msgs::Bool::ConstPtr& msg);

    // Method to assign goals to available TurtleBots
    void assignGoals();

    // Method to assign a goal to an available TurtleBot
    void assignGoalToAvailableBot(const geometry_msgs::PoseStamped &goal);

    // Method to publish the status of the bar robot (e.g., when the drink is ready to be deposited)
    void publishBarRobotStatus(const std::string &status);

    // Method to inform the TurtleBot that the drink has been deposited
    void informTurtleBotDrinkDeposited();
};

#endif // GOAL_MANAGER_H
