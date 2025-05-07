#include "people_detector.h"
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

PeopleDetector::PeopleDetector() : Node("people_detector"), hog_() {
    // Initialize HOG detector with the default people detector
    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    // Subscribe to the camera image topic
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&PeopleDetector::imageCallback, this, std::placeholders::_1)
    );

    // Subscribe to the Lidar topic
    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PeopleDetector::lidarCallback, this, std::placeholders::_1)
    );

    // Publisher for processed image
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/people_detector/image", 10
    );

    // Publisher for markers
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/people_marker", 10
    );
}

void PeopleDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat image;
    try {
        image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    processImage(image);

    // Convert to ROS2 image message and publish
    std_msgs::msg::Header header = msg->header;
    auto processed_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    image_publisher_->publish(*processed_msg);
}

void PeopleDetector::processImage(cv::Mat& image) {
    detected_people_.clear();
    hog_.detectMultiScale(image, detected_people_);

    // Draw bounding boxes
    for (const auto& rect : detected_people_) {
        cv::rectangle(image, rect, cv::Scalar(0, 255, 0), 2);
    }
}

void PeopleDetector::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Placeholder for lidar processing and determining positions
    people_positions_.clear();
    publishMarkers();
}

void PeopleDetector::publishMarkers() {
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < people_positions_.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "people";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = people_positions_[i];
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_array.markers.push_back(marker);
    }

    marker_publisher_->publish(marker_array);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PeopleDetector>());
    rclcpp::shutdown();
    return 0;
}
