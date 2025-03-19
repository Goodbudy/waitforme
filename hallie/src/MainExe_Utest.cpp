#include <rclcpp/rclcpp.hpp>
#include "TurtleBot.h"
#include "GoalManager.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node1 = std::make_shared<rclcpp::Node>("bot1_node");
    auto node2 = std::make_shared<rclcpp::Node>("bot2_node");
    // auto node3 = std::make_shared<rclcpp::Node>("bot3_node");
    // auto node4 = std::make_shared<rclcpp::Node>("bot4_node");
    // auto node5 = std::make_shared<rclcpp::Node>("bot5_node");

    TurtleBot bot1("bot1", node1);
    TurtleBot bot2("bot2", node2);
    // TurtleBot bot3("bot3", node3);
    // TurtleBot bot4("bot3", node4);
    // TurtleBot bot5("bot3", node5);

    auto goal_manager = std::make_shared<GoalManager>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node1);
    executor.add_node(node2);
    // executor.add_node(node3);
    // executor.add_node(node4);
    // executor.add_node(node5);
    executor.add_node(goal_manager);

    // Simulate status updates (in real case these would be dynamic)
    rclcpp::Rate rate(1.0);
    while (rclcpp::ok()) {
        bot1.setAtHome(true);
        bot1.setDelivering(true);
        bot1.setInProximity(true);
        bot1.publishStatus();

        bot2.setAtHome(false);
        bot2.setDelivering(true);
        bot2.setInProximity(true);
        bot2.publishStatus();

        // bot3.setAtHome(false);
        // bot3.setDelivering(false);
        // bot3.setInProximity(false);
        // bot3.publishStatus();

        // bot4.setAtHome(false);
        // bot4.setDelivering(false);
        // bot4.setInProximity(false);
        // bot4.publishStatus();

        // bot5.setAtHome(false);
        // bot5.setDelivering(false);
        // bot5.setInProximity(false);
        // bot5.publishStatus();

        executor.spin_once();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}