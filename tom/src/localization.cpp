#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class AutoLocalize : public rclcpp::Node {
public:
    AutoLocalize() : Node("auto_localize") {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&AutoLocalize::set_initial_pose, this));
        RCLCPP_INFO(this->get_logger(), "Localization Node Started");
    }

private:
//this sets the current coords to be 0,0 if robot is elsewhere this will not work.
    void set_initial_pose() {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.orientation.w = 1.0;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published initial pose");
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoLocalize>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
