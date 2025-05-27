#ifndef DETECTION_H
#define DETECTION_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For transform utilities with geometry_msgs
#include <tf2/utils.h> // For time utilities, if needed for transforms

class ObjDetect : public rclcpp::Node
{
public:
    ObjDetect();

private:

//////////////////////////////////hallie add//////////////////////////////////////////
    double resolution_;
    double origin_x_, origin_y_;
    int width_, height_;
    std::vector<int> grid_;  // 1 = obstacle, 0 = free
    bool map_loaded_ = false;
    void loadMapFromFile(const std::string& yaml_file);
    geometry_msgs::msg::Pose current_pose_;
//////////////////////////////////////////////////////////////////////
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    nav_msgs::msg::Odometry currentOdom;
    bool firstCent;
    std::vector<geometry_msgs::msg::Point> centres;
    int ct_;

    // Callbacks
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    // Object detection functions
    std::vector<std::vector<geometry_msgs::msg::Point>> countSegments(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void detectCylinder(const std::vector<geometry_msgs::msg::Point> &segment);
    void detectSquare(const std::vector<geometry_msgs::msg::Point> &segment);

    // Geometry and transformations
    geometry_msgs::msg::Point findCentre(geometry_msgs::msg::Point P1, geometry_msgs::msg::Point P2, double r);
    bool checkExisting(geometry_msgs::msg::Point centre);
    void visualizeSegment(const std::vector<geometry_msgs::msg::Point> &segment);
    visualization_msgs::msg::Marker produceMarkerCylinder(geometry_msgs::msg::Point pt, int type, const std::string& colour);

    // Transformations
    //geometry_msgs::msg::Point localToGlobal(const nav_msgs::msg::Odometry &odom, const geometry_msgs::msg::Point &local);
    //testing w/tf2
    geometry_msgs::msg::Point localToGlobal(const nav_msgs::msg::Odometry &global, const geometry_msgs::msg::Point &local);
    
    // Wall and corner detection
    bool isThisAWall(const std::vector<geometry_msgs::msg::Point> &segment);
    bool isThisACorner(const std::vector<geometry_msgs::msg::Point> &segment);
    bool isThis90(const std::vector<geometry_msgs::msg::Point> &segment);

    // Duplicate threshold for detection
    double duplicate_threshold_;

    //Testing w/tf2: TF2 buffer and listener
    // tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tf_listener_;

    //Testing w/ tf2: new function
    geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point &input_point, 
        const std::string &from_frame, 
        const std::string &to_frame);

    // Line fitting and angle calculation utilities
    /* std::pair<float, float> fitLine(
        const std::vector<geometry_msgs::msg::Point>& points, 
        size_t start, 
        size_t count);
    */
    /*float angleBetween(
        std::pair<float, float> v1, 
        std::pair<float, float> v2);
    */
};

#endif // DETECTION_H
