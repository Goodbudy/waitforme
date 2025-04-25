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

    auto goal1 = create_goal(1.5, -0.5);  // for tb1
    auto goal2 = create_goal(1.5, 0.5);   // for tb2

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

// #include <rclcpp/rclcpp.hpp>
// #include "TurtleBot.h"
// #include "GoalManager.h"
// #include <thread>

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);

//     // Nodes with namespaces that match those used in the launch file
//     auto tb1 = std::make_shared<rclcpp::Node>("tb1");
//     auto tb2 = std::make_shared<rclcpp::Node>("tb2");

//     // TurtleBot objects initialized with the same namespaces
//     auto bot1 = std::make_shared<TurtleBot>("tb1", tb1);
//     auto bot2 = std::make_shared<TurtleBot>("tb2", tb2);

//     auto goal_manager = std::make_shared<GoalManager>();
//     goal_manager->registerBot(bot1);
//     goal_manager->registerBot(bot2);

//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(tb1);
//     executor.add_node(tb2);
//     executor.add_node(goal_manager);

//     std::thread spin_thread([&executor]() {
//         executor.spin();
//     });

//     while (!bot1->isActionServerReady() || !bot2->isActionServerReady()) {
//         RCLCPP_INFO(tb1->get_logger(), "Waiting for both action servers...");
//         rclcpp::sleep_for(std::chrono::seconds(1));
//     }

//     goal_manager->sendGoal("tb1", 0.5, 0.8);
//     goal_manager->sendGoal("tb2", 0.2, 1);

//     rclcpp::sleep_for(std::chrono::seconds(20));
//     rclcpp::shutdown();
//     spin_thread.join();

//     return 0;
// }

///////////////////////////////////////////////////////////////////////////////

// #include <rclcpp/rclcpp.hpp>
// #include "TurtleBot.h"
// #include "GoalManager.h"
// #include <thread>

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);

//     auto turtlebot_node = std::make_shared<rclcpp::Node>("tb3_1");
//     auto bot1 = std::make_shared<TurtleBot>("bot1", turtlebot_node);
//     auto goal_manager = std::make_shared<GoalManager>(bot1);

//     // Set sim time
//     if (!turtlebot_node->has_parameter("use_sim_time")) {
//         turtlebot_node->declare_parameter("use_sim_time", rclcpp::ParameterValue(true));
//     }
//     turtlebot_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    
//     //turtlebot_node->declare_parameter("use_sim_time", rclcpp::ParameterValue(true));
//     //turtlebot_node->set_parameter(rclcpp::Parameter("use_sim_time", true));

//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(turtlebot_node);
//     executor.add_node(goal_manager);

//     std::thread spin_thread([&executor]() {
//         executor.spin();
//     });

//     // ✅ Wait until action server is up before sending the goal
//     while (!bot1->isActionServerReady()) {
//         RCLCPP_INFO(turtlebot_node->get_logger(), "[main] Waiting for action server...");
//         rclcpp::sleep_for(std::chrono::seconds(1));
//     }

//     // ✅ Now send the goal automatically
//     goal_manager->sendGoal(1.0, 1.0);

//     rclcpp::sleep_for(std::chrono::seconds(15));
//     rclcpp::shutdown();
//     spin_thread.join();

//     return 0;
// }
