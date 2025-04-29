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

void run_phase(
    const geometry_msgs::msg::PoseStamped& start_pose,
    const geometry_msgs::msg::PoseStamped& goal_pose,
    const std::shared_ptr<GoalManager>& manager,
    const std::shared_ptr<TurtleBot>& bot)
{
    bot->setHomePosition(start_pose.pose.position.x, start_pose.pose.position.y);
    bot->setAtHome(true);  // Reset to home status

    manager->register_bot(bot->getName(), bot);
    manager->queue_goal(bot->getName(), goal_pose);

    rclcpp::Rate rate(10.0); // 10Hz update rate
    while (rclcpp::ok() && !(bot->isHome() && !manager->has_pending_goals(bot->getName()))) {
        bot->publishStatus();  // Optional: Keep publishing status during motion
        manager->update();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto manager = std::make_shared<GoalManager>();
    auto node = std::make_shared<rclcpp::Node>("tb2_node");
    auto bot = std::make_shared<TurtleBot>("tb2", node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(manager);
    executor.add_node(node);

    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    geometry_msgs::msg::PoseStamped home = create_goal(3.5, 3.2, 0.0);
    std::vector<geometry_msgs::msg::PoseStamped> task_goals = {
        create_goal(2.0, 2.5, 0.0),
        create_goal(1.0, 0.8, 0.0)
    };

    geometry_msgs::msg::PoseStamped current_pose = home;

    for (size_t i = 0; i < task_goals.size(); ++i) {
        RCLCPP_INFO(rclcpp::get_logger("main"), "--- PHASE %zu: Going to task ---", 2 * i + 1);
        run_phase(current_pose, task_goals[i], manager, bot);

        RCLCPP_INFO(rclcpp::get_logger("main"), "--- PHASE %zu: Returning home ---", 2 * i + 2);
        run_phase(task_goals[i], home, manager, bot);

        current_pose = home;
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Finished all goal phases.");

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}

