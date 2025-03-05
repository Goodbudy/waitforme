#ifndef TURTLEBOT_H
#define TURTLEBOT_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

class TurtleBot {
public:
    std::string bot_name;  // The name of the TurtleBot
    bool is_home;          // Whether the bot is at home
    geometry_msgs::PoseStamped current_position;  // Current position of the bot

    // Constructor to initialize the bot name and set the bot as not at home
    TurtleBot(std::string name) : bot_name(name), is_home(false) {}

    // Simulate receiving a goal
    void receiveGoal(const geometry_msgs::PoseStamped &goal);

    // Simulate moving the robot to the goal
    void moveToGoal(const geometry_msgs::PoseStamped &goal);

    // Simulate returning the robot to home base
    void returnToHome();

    // Check if the robot is at home
    bool isAtHome();

    // Simulate periodic checks to see if the robot is at home
    void checkIfAtHome();
};

#endif // TURTLEBOT_H
