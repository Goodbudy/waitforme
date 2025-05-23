#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <chrono>
#include <thread>

class AutoLocalise : public rclcpp::Node {
public:
    // AutoLocalise() : Node("localisation_node_initial") {
    //     this->declare_parameter("use_sim_time", true);  // make sure sim time is active
    //     publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    //     timer_ = this->create_wall_timer(
    //         std::chrono::seconds(2),
    //         std::bind(&AutoLocalise::set_initial_pose, this)
    //     );
    // }

    AutoLocalise() : Node("localisation_node_initial") {
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", true);  // Declare only if not already declared
        }
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        odom_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/odom", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&AutoLocalise::set_initial_pose, this)
        );
        
    }
    

private:
    void set_initial_pose() {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
       // msg.pose.pose.position.x = 3.12;
       // msg.pose.pose.position.y = 3.31;
        msg.pose.pose.position.x = 0.00;
        msg.pose.pose.position.y = 0.00;
        msg.pose.pose.orientation.w = 1.0;

        // Minimal 6x6 covariance matrix for x, y, and yaw
        for (int i = 0; i < 36; ++i) msg.pose.covariance[i] = 0.0;
        msg.pose.covariance[0] = 0.25;         // x
        msg.pose.covariance[7] = 0.25;         // y
        msg.pose.covariance[35] = 0.0685;      // yaw

        publisher_->publish(msg);
        odom_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "✅ Published initial pose");

        // Shutdown after publishing
        rclcpp::shutdown();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoLocalise>());  // spin properly
    return 0;
}
