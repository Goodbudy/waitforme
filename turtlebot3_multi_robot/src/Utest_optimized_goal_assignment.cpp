#include <rclcpp/rclcpp.hpp>
#include "GoalManager.h"
#include "TurtleBot.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <vector>
#include <memory>

geometry_msgs::msg::PoseStamped create_goal(double x, double y, double yaw = 0.0) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = rclcpp::Clock().now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.w = 1.0;  // Facing forward
    return goal;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto manager = std::make_shared<GoalManager>();
    auto node1 = std::make_shared<rclcpp::Node>("tb1_node");
    auto node2 = std::make_shared<rclcpp::Node>("tb2_node");
    auto tb1 = std::make_shared<TurtleBot>("tb1", node1);
    auto tb2 = std::make_shared<TurtleBot>("tb2", node2);

    tb1->setHomePosition(3.0, 3.0);
    tb2->setHomePosition(1.0, 3.0);
    tb1->setAtHome(true);
    tb2->setAtHome(true);

    manager->register_bot(tb1->getName(), tb1);
    manager->register_bot(tb2->getName(), tb2);

    std::vector<geometry_msgs::msg::PoseStamped> goals = {
        create_goal(2.0, 2.5),
        create_goal(1.0, 0.8),
        create_goal(3.0, 1.8),
        create_goal(2.0, 2.5)
    };

    for (const auto& g : goals) {
        manager->queue_global_goal(g);
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(manager);
    executor.add_node(node1);
    executor.add_node(node2);

    std::thread exec_thread([&executor]() {
        executor.spin();
    });

    rclcpp::Rate rate(10.0);
    while (rclcpp::ok()) {
        manager->update();
        tb1->publishStatus();
        tb2->publishStatus();

        if (!manager->has_global_goals() && tb1->isHome() && tb2->isHome()) {
            break;
        }

        rate.sleep();
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "All goals completed.");
    rclcpp::shutdown();
    exec_thread.join();
    return 0;
}
