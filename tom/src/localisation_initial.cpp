// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// #include <chrono>
// #include <thread>

// class AutoLocalise : public rclcpp::Node {
// public:
//     // AutoLocalise() : Node("localisation_node_initial") {
//     //     this->declare_parameter("use_sim_time", true);  // make sure sim time is active
//     //     publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
//     //     timer_ = this->create_wall_timer(
//     //         std::chrono::seconds(2),
//     //         std::bind(&AutoLocalise::set_initial_pose, this)
//     //     );
//     // }

//     AutoLocalise() : Node("localisation_node_initial") {
//         if (!this->has_parameter("use_sim_time")) {
//             this->declare_parameter("use_sim_time", true);  // Declare only if not already declared
//         }
//         publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
//         odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(2),
//             std::bind(&AutoLocalise::set_initial_pose, this)
//         );
        
//     }
    

// private:
//     void set_initial_pose() {
//         auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
//         msg.header.stamp = this->get_clock()->now();
//         msg.header.frame_id = "map";
//        // msg.pose.pose.position.x = 3.12;
//        // msg.pose.pose.position.y = 3.31;
//         msg.pose.pose.position.x = 0.00;
//         msg.pose.pose.position.y = 0.00;
//         msg.pose.pose.orientation.w = 1.0;

//         // Minimal 6x6 covariance matrix for x, y, and yaw
//         for (int i = 0; i < 36; ++i) msg.pose.covariance[i] = 0.0;
//         msg.pose.covariance[0] = 0.25;         // x
//         msg.pose.covariance[7] = 0.25;         // y
//         msg.pose.covariance[35] = 0.0685;      // yaw

//         publisher_->publish(msg);
//         odom_publisher_->publish(msg);
//         RCLCPP_INFO(this->get_logger(), "✅ Published initial pose");

//         // Shutdown after publishing
//         rclcpp::shutdown();
//     }

//     rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
//     rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<AutoLocalise>());  // spin properly
//     return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>

class AutoLocalise : public rclcpp::Node {
public:
    AutoLocalise() : Node("localisation_node_initial") {
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", true);
        }
        initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&AutoLocalise::set_initial_pose, this)
        );
    }

private:
    void set_initial_pose() {
        // Prepare PoseWithCovarianceStamped for /initialpose
        auto initial_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        initial_pose_msg.header.stamp = this->get_clock()->now();
        initial_pose_msg.header.frame_id = "map";
        initial_pose_msg.pose.pose.position.x = 0.5;
        initial_pose_msg.pose.pose.position.y = -1.2;
        initial_pose_msg.pose.pose.position.z = 0.0;
        initial_pose_msg.pose.pose.orientation.w = 1.0;
        initial_pose_msg.pose.pose.orientation.x = 0.0;
        initial_pose_msg.pose.pose.orientation.y = 0.0;
        initial_pose_msg.pose.pose.orientation.z = 0.0;

        for (int i = 0; i < 36; ++i) initial_pose_msg.pose.covariance[i] = 0.0;
        initial_pose_msg.pose.covariance[0] = 0.25;    // x variance
        initial_pose_msg.pose.covariance[7] = 0.25;    // y variance
        initial_pose_msg.pose.covariance[35] = 0.0685; // yaw variance

        // Prepare Odometry message for /odom
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = initial_pose_msg.header.stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Copy pose info
        odom_msg.pose.pose = initial_pose_msg.pose.pose;
        for (int i = 0; i < 36; ++i) odom_msg.pose.covariance[i] = initial_pose_msg.pose.covariance[i];

        // Twist zero since no velocity given
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        initialpose_pub_->publish(initial_pose_msg);
        odom_pub_->publish(odom_msg);

        RCLCPP_INFO(this->get_logger(), "✅ Published initial pose and odometry");

        rclcpp::shutdown();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoLocalise>());
    return 0;
}
