#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cmath>
#include <iostream>

class ProofOfConcept : public rclcpp::Node {
public:
    ProofOfConcept(double goal_x, double goal_y)
        : Node("proofofconcept"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ProofOfConcept::laser_callback, this, std::placeholders::_1));

        // Set the goal pose based on input coordinates
        goal_pose_.pose.position.x = goal_x;
        goal_pose_.pose.position.y = goal_y;
        goal_received_ = true;

        RCLCPP_INFO(this->get_logger(), "Goal set to: (%.2f, %.2f)", goal_pose_.pose.position.x, goal_pose_.pose.position.y);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ProofOfConcept::control_loop, this));
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        min_distance_ = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    }

    void control_loop() {
        if (!goal_received_) return;

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform("base_link", "map", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }

        geometry_msgs::msg::PoseStamped goal_in_base;
        tf2::doTransform(goal_pose_, goal_in_base, transform);

        double distance = std::hypot(goal_in_base.pose.position.x, goal_in_base.pose.position.y);
        double angle = std::atan2(goal_in_base.pose.position.y, goal_in_base.pose.position.x);

        geometry_msgs::msg::Twist cmd_vel;

        if (distance > 0.1) {
            if (min_distance_ < 0.5) {  // Obstacle detected
                cmd_vel.angular.z = 0.5;  // Rotate to avoid obstacle
                cmd_vel.linear.x = 0.0;
            } else {
                cmd_vel.linear.x = 0.2;
                cmd_vel.angular.z = angle * 0.5;
            }
        } else {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Goal reached.");
            goal_received_ = false;
        }

        cmd_vel_pub_->publish(cmd_vel);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    geometry_msgs::msg::PoseStamped goal_pose_;
    bool goal_received_ = false;
    double min_distance_ = std::numeric_limits<double>::infinity();
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Check if the user provided coordinates for the goal
    if (argc != 3) {
        std::cerr << "Usage: ros2 run issy proofofconcept <goal_x> <goal_y>" << std::endl;
        return 1;
    }

    double goal_x = std::stod(argv[1]);
    double goal_y = std::stod(argv[2]);

    auto node = std::make_shared<ProofOfConcept>(goal_x, goal_y);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
