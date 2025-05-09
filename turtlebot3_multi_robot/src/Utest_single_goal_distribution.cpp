#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "GoalManager.h"

geometry_msgs::msg::PoseStamped createGoal(double x, double y) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = rclcpp::Clock().now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.w = 1.0;
    return goal;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto goal_manager = std::make_shared<GoalManager>();

    auto goal1 = createGoal(1.0, 2.0);
    auto goal2 = createGoal(3.0, 4.0);

    goal_manager->queue_global_goal(goal1);
    goal_manager->queue_global_goal(goal2);

    if (!goal_manager->has_global_goals()) {
        RCLCPP_ERROR(goal_manager->get_logger(), "Expected global goals in queue but found none.");
        return 1;
    }

    // Use MultiThreadedExecutor to simulate threading
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(goal_manager);

    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
        executor.spin_some();
    }

    if (!goal_manager->has_global_goals()) {
        RCLCPP_ERROR(goal_manager->get_logger(), "Global goals unexpectedly disappeared from queue.");
        return 1;
    }

    executor.remove_node(goal_manager);
    RCLCPP_INFO(goal_manager->get_logger(), "GoalManager test passed as executable.");

    rclcpp::shutdown();
    return 0;
}
