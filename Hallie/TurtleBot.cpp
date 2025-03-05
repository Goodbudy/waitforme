#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <iostream>

class TurtleBot {
public:
    std::string bot_name;
    bool is_home;
    geometry_msgs::PoseStamped current_position;

    TurtleBot(std::string name) : bot_name(name), is_home(false) {}

    // Simulate receiving a goal
    void receiveGoal(const geometry_msgs::PoseStamped &goal) {
        // Print confirmation that the goal has been received
        std::cout << bot_name << " received goal: [" 
                  << goal.pose.position.x << ", " 
                  << goal.pose.position.y << "]" << std::endl;
        
        // Simulate moving to the goal
        moveToGoal(goal);
    }

    // Simulate moving the robot to the goal
    void moveToGoal(const geometry_msgs::PoseStamped &goal) {
        // Simulate movement logic
        std::cout << bot_name << " is moving to goal: [" 
                  << goal.pose.position.x << ", " 
                  << goal.pose.position.y << "]" << std::endl;

        // Update the position
        current_position = goal;

        // Simulate that bot is not at home while moving
        is_home = false;

        // After reaching the goal, print confirmation and update is_home flag
        std::cout << bot_name << " has reached goal at [" 
                  << current_position.pose.position.x << ", " 
                  << current_position.pose.position.y << "]" << std::endl;

        // Simulate the bot returning home
        returnToHome();
    }

    // Simulate returning the robot to home base
    void returnToHome() {
        std::cout << bot_name << " is returning home." << std::endl;

        // Simulate that bot is not home while returning
        is_home = false;

        // Assuming the bot has returned to home within a tolerance, set it to home
        std::cout << bot_name << " has returned to home within tolerance." << std::endl;
        is_home = true;
    }

    bool isAtHome() {
        return is_home;
    }

    // Simulate periodic checks for whether the bot is at home or not
    void checkIfAtHome() {
        if (is_home) {
            std::cout << bot_name << " is at home and ready for a new task." << std::endl;
        } else {
            std::cout << bot_name << " is not at home yet." << std::endl;
        }
    }
};
