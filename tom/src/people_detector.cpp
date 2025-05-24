#include "people_detector.h"
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

PeopleDetector::PeopleDetector() : Node("people_detector"), hog_() {
    // Initialize HOG detector with the default people detector
    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    // === Webcam initialization (commented out for TurtleBot3 camera usage) ===
    /////WEB1START
    
    webcam_.open("/dev/video0", cv::CAP_V4L2);  // Use V4L2 explicitly
    if (!webcam_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open webcam");
    }

    int backend = webcam_.get(cv::CAP_PROP_BACKEND);
    RCLCPP_INFO(this->get_logger(), "OpenCV Video Capture Backend: %d", backend);

    if (backend == cv::CAP_V4L2) {
        RCLCPP_INFO(this->get_logger(), "Using V4L2 backend for video capture");
    } else {
        RCLCPP_WARN(this->get_logger(), "Not using V4L2 backend, it might cause issues");
    }
    
    /////WEB1END/ROB1START
    /*
    // Subscribe to TurtleBot3 camera image topic
    
    // image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
    //     "/camera/image_raw/compressed", 10,
    //     std::bind(&PeopleDetector::imageCallback, this, std::placeholders::_1)    
    // );

    image_transport::ImageTransport it(std::enable_shared_from_this<PeopleDetector>::shared_from_this());
    image_subscriber_ = it.subscribe(
    "/camera/image_raw",
    10,
    std::bind(&PeopleDetector::imageCallback, std::enable_shared_from_this<PeopleDetector>::shared_from_this(), std::placeholders::_1),
    image_transport::TransportHints(this, "image_transport", "compressed")
);

*/

    //////ROB1END

    // Publisher for processed image
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/people_detector/image", 10
    );

    // Publisher for markers
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/people_marker", 10
    );
}

////ROB2START
/*
void PeopleDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat image;
    try {
        image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    processImage(image);

    std_msgs::msg::Header header = msg->header;
    header.frame_id = "camera_link";  // or "base_link" depending on your TF tree
    auto processed_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    image_publisher_->publish(*processed_msg);
}
*/
////WEB2START

// === Webcam capture loop (commented out) ===
void PeopleDetector::captureAndProcessImage() {
    RCLCPP_INFO(this->get_logger(), "Capturing webcam frame...");
    cv::Mat frame;
    webcam_ >> frame;

    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty frame captured from webcam");
        return;
    }
    else {
    RCLCPP_INFO(this->get_logger(), "Frame captured successfully");
    }
    
    processImage(frame);

    std_msgs::msg::Header header;
    header.stamp = this->now();
    auto processed_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    image_publisher_->publish(*processed_msg);
}

////WEB2END

void PeopleDetector::processImage(cv::Mat& image) {
    detected_people_.clear();
    hog_.detectMultiScale(image, detected_people_);

    //draw bounding boxes
    for (const auto& rect : detected_people_) {
        cv::rectangle(image, rect, cv::Scalar(0, 255, 0), 2);
    }
}

/*
void PeopleDetector::publishMarkers() {
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < people_positions_.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "camera_link";  // or "base_link"
        marker.header.stamp = this->now();
        marker.ns = "people";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
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
*/

/////ROBO3START
/*
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PeopleDetector>());
    rclcpp::shutdown();
    return 0;
}
*/
/////ROBO3END/WEB3START

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PeopleDetector>();

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
        node->captureAndProcessImage();  // This is critical
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

////WEB3END

