#ifndef DETECTION_H
#define DETECTION_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>

class ObjDetect : public rclcpp::Node
{
public:
    ObjDetect();

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;


    nav_msgs::msg::Odometry currentOdom;
    bool firstCent;
    std::vector<geometry_msgs::msg::Point> centres;
    int ct_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    std::vector<std::vector<geometry_msgs::msg::Point>> countSegments(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void detectCylinder(const std::vector<geometry_msgs::msg::Point> &segment);
    void detectSquare(const std::vector<geometry_msgs::msg::Point> &segment);
    geometry_msgs::msg::Point findCentre(geometry_msgs::msg::Point P1, geometry_msgs::msg::Point P2, double r);
    bool checkExisting(geometry_msgs::msg::Point centre);
    void visualizeSegment(const std::vector<geometry_msgs::msg::Point> &segment);
    visualization_msgs::msg::Marker produceMarkerCylinder(geometry_msgs::msg::Point pt, int type, const std::string& colour);
    geometry_msgs::msg::Point localToGlobal(const nav_msgs::msg::Odometry &global, const geometry_msgs::msg::Point &local);
    bool isThisAWall(const std::vector<geometry_msgs::msg::Point> &segment);
    bool isThisACorner(const std::vector<geometry_msgs::msg::Point> &segment);
    bool isThis90(const std::vector<geometry_msgs::msg::Point> &segment);
    double duplicate_threshold_;

//   std::pair<float, float> fitLine1(const std::vector<geometry_msgs::msg::Point>& segment);
//   float angleBetweenSegments(const std::vector<geometry_msgs::msg::Point>& seg1, const std::vector<geometry_msgs::msg::Point>& seg2);
// Returns a unit direction vector as a pair (dx, dy)
std::pair<float, float> fitLine(
    const std::vector<geometry_msgs::msg::Point>& points, 
    size_t start, 
    size_t count);

// Returns angle between two 2D unit vectors in degrees
float angleBetween(
    std::pair<float, float> v1, 
    std::pair<float, float> v2);
};

#endif // DETECTION_H
