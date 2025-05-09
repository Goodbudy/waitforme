#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <vector>
#include <memory>
#include "GoalManager.h"

geometry_msgs::msg::PoseStamped create_goal(double x, double y, double yaw = 0.0) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = rclcpp::Clock().now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.w = 1.0;
    return goal;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto manager = std::make_shared<GoalManager>();

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

    std::thread exec_thread([&executor]() {
        executor.spin();
    });

    RCLCPP_INFO(rclcpp::get_logger("main"), "Goals queued. Running until all are assigned and completed.");

    // Monitor until queue is empty (i.e., all goals assigned and completed)
    while (rclcpp::ok() && manager->has_global_goals()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "All goals assigned. Waiting for bots to return home.");

    // Let simulation complete final movements back to home
    std::this_thread::sleep_for(std::chrono::seconds(10));

    RCLCPP_INFO(rclcpp::get_logger("main"), "Finished optimized goal assignment test.");
    rclcpp::shutdown();
    exec_thread.join();
    return 0;
}
