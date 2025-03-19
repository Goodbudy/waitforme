#include "path_planner.hpp"

class PathPlannerNode : public rclcpp::Node, public std::enable_shared_from_this<PathPlannerNode>{
    public:
    PathPlannerNode() : Node("path_planner_node") {
         // Now that the node is fully constructed, we can use shared_from_this()

        // Example: Teammates can call this directly in their code
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.pose.position.x = 2.0;
        goal.pose.position.y = 1.0;
        goal.pose.orientation.w = 1.0;

        // Call the send_goal function of PathPlanner class
        path_planner_->send_goal(std::make_shared<geometry_msgs::msg::PoseStamped>(goal));
    }
    
    
    
    private:
        std::shared_ptr<PathPlanner> path_planner_;
    };
    
    int main(int argc, char **argv) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<PathPlannerNode>());
        rclcpp::shutdown();
        return 0;
    }