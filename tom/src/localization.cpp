#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <chrono>
#include <thread>  // For sleep_for()

class AutoLocalize : public rclcpp::Node {
public:
    AutoLocalize() : Node("auto_localize") {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        std::this_thread::sleep_for(std::chrono::seconds(1)); // Ensure publisher is fully initialized
        set_initial_pose();
        RCLCPP_INFO(this->get_logger(), "Localization Node Started");
    }

private:
    void set_initial_pose() {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.orientation.w = 1.0;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published initial pose");

        // Give time for message to be sent before node shuts down
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoLocalize>();

    rclcpp::spin_some(node);  // Allow processing of published message
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Ensure publish is completed

    rclcpp::shutdown();
    return 0;
}
