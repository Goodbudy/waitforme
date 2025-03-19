// Test Criteria - Pass
#include <rclcpp/rclcpp.hpp>
#include "GoalManager.h"
#include "TurtleBot.h"
#include <chrono>
#include <thread>
#include <unordered_map>
#include <iostream>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto bot1_node = std::make_shared<rclcpp::Node>("bot1_node");
    auto bot2_node = std::make_shared<rclcpp::Node>("bot2_node");

    TurtleBot bot1("bot1", bot1_node);
    TurtleBot bot2("bot2", bot2_node);

    auto goal_manager = std::make_shared<GoalManager>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(bot1_node);
    executor.add_node(bot2_node);
    executor.add_node(goal_manager);

    // Simulate both bots at home and available
    bot1.setAtHome(true);
    bot1.setDelivering(false);
    bot1.setInProximity(true);

    bot2.setAtHome(true);
    bot2.setDelivering(false);
    bot2.setInProximity(true);

    // Publish their statuses
    bot1.publishStatus();
    bot2.publishStatus();

    // Allow some time for subscriptions to update
    std::this_thread::sleep_for(std::chrono::seconds(1));
    executor.spin_once();

    // Allow goal manager's timer to trigger at least once
    std::this_thread::sleep_for(std::chrono::seconds(6));
    executor.spin_some();

    // Check log manually for now:
    std::cout << "[Test] Please check logs to confirm that:" << std::endl;
    std::cout << "- Both bot1 and bot2 received DIFFERENT goals" << std::endl;
    std::cout << "- If they both received the SAME goal or only one received a goal, test FAILS" << std::endl;

    rclcpp::shutdown();
    return 0;
}
