#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <vector>
#include <memory>
#include <functional>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class MovementLogic : public rclcpp::Node {
public:
    MovementLogic() : Node("movementlogic") {
        this->declare_parameter("home_x", 0.0);
        this->declare_parameter("home_y", 0.0);
        home_x_ = this->get_parameter("home_x").as_double();
        home_y_ = this->get_parameter("home_y").as_double();

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Example goal list for demonstration
        goals_ = { {1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}, {7.0, 8.0}, {9.0, 10.0} };

        moveTo(home_x_, home_y_, [this]() {
            RCLCPP_INFO(this->get_logger(), "Arrived at home base.");
            moveThroughGoals();
        });
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    std::vector<std::pair<double, double>> goals_;
    double home_x_, home_y_;
    size_t current_goal_index_ = 0;

    void moveTo(double x, double y, std::function<void()> callback) {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [callback](auto) { callback(); };
        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void moveThroughGoals() {
        if (current_goal_index_ < goals_.size()) {
            auto [x, y] = goals_[current_goal_index_++];
            moveTo(x, y, [this]() {
                RCLCPP_INFO(this->get_logger(), "Arrived at goal.");
                rclcpp::sleep_for(15s);
                moveTo(home_x_, home_y_, [this]() {
                    RCLCPP_INFO(this->get_logger(), "Returned to home base.");
                    moveThroughGoals();
                });
            });
        } else {
            RCLCPP_INFO(this->get_logger(), "All goals completed.");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementLogic>());
    rclcpp::shutdown();
    return 0;
}
