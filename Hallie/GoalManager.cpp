#include <iostream>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

class GoalManager {
private:
    std::vector<TurtleBot> turtleBots;
    std::queue<geometry_msgs::PoseStamped> goalQueue;
    std::queue<geometry_msgs::PoseStamped> urgentGoalQueue;
    ros::Publisher barRobotPub;  // Publisher to inform the bar robot that the drink is ready
    ros::Subscriber botStatusSub;  // Subscriber for updates on TurtleBot status

public:
    GoalManager(std::vector<TurtleBot> &bots, ros::NodeHandle &nh) : turtleBots(bots) {
        barRobotPub = nh.advertise<std_msgs::String>("bar_robot_status", 10);
        botStatusSub = nh.subscribe("turtlebot_status", 10, &GoalManager::updateBotStatus, this);
    }

    void addGoalToQueue(const geometry_msgs::PoseStamped &goal, bool isUrgent = false) {
        if (isUrgent) {
            urgentGoalQueue.push(goal);  // Urgent goals go to the front of the queue
        } else {
            goalQueue.push(goal);  // Non-urgent goals are added in the order they come
        }
    }

    void updateBotStatus(const std_msgs::Bool::ConstPtr& msg) {
        // Assuming the msg indicates if the TurtleBot is ready for a new goal
        if (msg->data) {  // If the bot is at home and ready
            assignGoals();
        }
    }

    void assignGoals() {
        while (!urgentGoalQueue.empty() || !goalQueue.empty()) {
            // First, process all urgent goals
            if (!urgentGoalQueue.empty()) {
                geometry_msgs::PoseStamped urgentGoal = urgentGoalQueue.front();
                urgentGoalQueue.pop();
                assignGoalToAvailableBot(urgentGoal);
            }

            // Then process normal goals
            if (!goalQueue.empty()) {
                geometry_msgs::PoseStamped goal = goalQueue.front();
                goalQueue.pop();
                assignGoalToAvailableBot(goal);
            }
        }
    }

    void assignGoalToAvailableBot(const geometry_msgs::PoseStamped &goal) {
        // Find the first available bot to assign the goal
        for (auto &bot : turtleBots) {
            if (bot.isHome()) {
                bot.receiveGoal(goal);
                ROS_INFO("%s assigned to goal: [%f, %f]", bot.bot_name.c_str(), goal.pose.position.x, goal.pose.position.y);
                
                // After the goal is assigned, the bot goes to pick up the drink
                // Assuming after pickup, it will inform the bar robot to deposit the drink
                publishBarRobotStatus("Drink is ready to be deposited");
                informTurtleBotDrinkDeposited();
                return;  // Goal assigned, exit the function
            }
        }
        // If no bot is available, the goal will stay in the queue until a bot becomes free
        ROS_WARN("No available bot, goal [%f, %f] is waiting.", goal.pose.position.x, goal.pose.position.y);
        goalQueue.push(goal);  // Re-add the goal to the queue
    }

    void publishBarRobotStatus(const std::string &status) {
        std_msgs::String msg;
        msg.data = status;
        barRobotPub.publish(msg);
        ROS_INFO("Published status to Bar Robot: %s", status.c_str());
    }

    void informTurtleBotDrinkDeposited() {
        ROS_INFO("Informing TurtleBot that the drink has been deposited.");
        // You would add any specific logic here to communicate with the TurtleBot
    }
};
