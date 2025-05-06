#include <rclcpp/rclcpp.hpp>
#include "GoalManager.h"

geometry_msgs::msg::PoseStamped create_goal(double x, double y, double yaw = 0.0) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = rclcpp::Clock().now();

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;

    goal.pose.orientation.w = 1.0; // basic forward-facing

    return goal;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // auto node = rclcpp::Node::make_shared("goal_test_node");
    auto manager = std::make_shared<GoalManager>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(manager);
    
    std::thread executor_thread([&executor](){
        executor.spin();
    });

    auto goal1 = create_goal(2.0, 2.5);  // for tb1
    auto goal2 = create_goal(1.0, 0.8);   // for tb2

    rclcpp::sleep_for(std::chrono::seconds(5));

    manager->send_goal("tb1", goal1);
    
    rclcpp::sleep_for(std::chrono::seconds(5));
    
    manager->send_goal("tb2", goal2);
    
    // rclcpp::sleep_for(std::chrono::seconds(20));
    // rclcpp::spin(node);
    //rclcpp::shutdown();
    executor_thread.join(); 
    return 0;
}

