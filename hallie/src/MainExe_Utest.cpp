#include <iostream>
#include <vector>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include "TurtleBot.h"  // Include the TurtleBot header
#include "GoalManager.h"  // Include the GoalManager header (assuming this exists)

// Main function where everything ties together
int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_manager_node");
    ros::NodeHandle nh;

    // Create and initialize TurtleBots
    std::vector<TurtleBot> bots = {TurtleBot("TurtleBot_1"), TurtleBot("TurtleBot_2")};

    // Initialize GoalManager with ROS NodeHandle
    GoalManager manager(bots, nh);

    // Simulate an external process that sends orders/goals (simulating a host or system)
    ros::Rate loop_rate(1);  // 1 Hz loop rate, simulate new orders coming in every second

    while (ros::ok()) {
        // In a real scenario, these would be data from external sources, like a service or topic
        geometry_msgs::PoseStamped newGoal;
        newGoal.pose.position.x = rand() % 10;  // Random x position for the goal
        newGoal.pose.position.y = rand() % 10;  // Random y position for the goal
        newGoal.pose.position.z = 0.0;

        // Simulate the type of goal (normal or urgent)
        bool isUrgent = (rand() % 2 == 0);  // Randomly determine if the goal is urgent or not

        ROS_INFO("New goal received: [%f, %f] (Urgent: %s)", newGoal.pose.position.x, newGoal.pose.position.y, isUrgent ? "Yes" : "No");

        // Add goal to the manager
        manager.addGoalToQueue(newGoal, isUrgent);

        // Process goals and assign them to available TurtleBots
        manager.assignGoals();

        // Simulate the TurtleBot receiving the goal and moving
        for (auto &bot : bots) {
            if (bot.isAtHome()) {
                bot.receiveGoal(newGoal);  // Assign the new goal to the bot
            }
        }

        // Check if any TurtleBots are home and ready for new goals
        for (auto &bot : bots) {
            bot.checkIfAtHome();  // Periodically check if the bot is at home
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
