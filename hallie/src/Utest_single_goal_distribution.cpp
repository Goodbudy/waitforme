#include <rclcpp/rclcpp.hpp>
#include "TurtleBot.h"
#include "GoalManager.h"
#include <thread>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto bot1_node = std::make_shared<rclcpp::Node>("bot1_node");
    auto bot2_node = std::make_shared<rclcpp::Node>("bot2_node");

    auto bot1 = std::make_shared<TurtleBot>("bot1", bot1_node);
    auto bot2 = std::make_shared<TurtleBot>("bot2", bot2_node);

    auto goal_manager = std::make_shared<GoalManager>();
    goal_manager->registerBot(bot1);
    goal_manager->registerBot(bot2);    

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(bot1_node);
    executor.add_node(bot2_node);
    executor.add_node(goal_manager);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    while (!bot1->isActionServerReady() || !bot2->isActionServerReady()) {
        RCLCPP_INFO(bot1_node->get_logger(), "Waiting for both action servers...");
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    goal_manager->sendGoal("bot1", 0.5, 0.8);
    goal_manager->sendGoal("bot2", 0.2, 1);    

    rclcpp::sleep_for(std::chrono::seconds(20));
    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}


///////////////////////////////////////////////////////////////////////////////

// #include <rclcpp/rclcpp.hpp>
// #include "TurtleBot.h"
// #include "GoalManager.h"
// #include <thread>

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);

//     auto turtlebot_node = std::make_shared<rclcpp::Node>("bot1_node");
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
