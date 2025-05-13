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
    goal.pose.orientation.w = 1.0;  // default facing forward
    return goal;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto manager = std::make_shared<GoalManager>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(manager);

    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    std::vector<geometry_msgs::msg::PoseStamped> task_goals = {
        create_goal(2.0, 2.5),
        create_goal(1.0, 0.8)
    };

    RCLCPP_INFO(rclcpp::get_logger("main"), "--- Robot starting at home ---");

    for (const auto& goal : task_goals) {
        manager->queue_global_goal(goal);
    }

    // Wait until all goals are dispatched
    while (rclcpp::ok() && manager->has_global_goals()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "All task goals assigned. Waiting for final homing...");

    std::this_thread::sleep_for(std::chrono::seconds(10));

    RCLCPP_INFO(rclcpp::get_logger("main"), "Finished goal reassignment sequence.");

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}
