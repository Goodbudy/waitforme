
#include "detection.h"
#include <cmath>
#include <utility>
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <std_msgs/msg/color_rgba.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>


ObjDetect::ObjDetect() : Node("detection_node"), firstCent(true), ct_(0)
{
    // Adjust QoS for the scan subscription
    rclcpp::QoS qos_profile{rclcpp::SensorDataQoS()};
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); // or RELIABLE

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos_profile, std::bind(&ObjDetect::scanCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&ObjDetect::odomCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/visualization_marker", 10);
}

//testing w/ tf2
/*
ObjDetect::ObjDetect() : Node("detection_node"), firstCent(true), ct_(0), 
    tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    // Adjust QoS for the scan subscription
    rclcpp::QoS qos_profile{rclcpp::SensorDataQoS()};
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); // or RELIABLE

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos_profile, std::bind(&ObjDetect::scanCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&ObjDetect::odomCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/visualization_marker", 10);
}
*/
/*
//testing w/ tf2 :transform between frames
geometry_msgs::msg::Point ObjDetect::transformPoint(const geometry_msgs::msg::Point &input_point, 
    const std::string &from_frame, 
    const std::string &to_frame)
{
    geometry_msgs::msg::PointStamped input_point_stamped;
    input_point_stamped.header.frame_id = from_frame;
    input_point_stamped.header.stamp = this->get_clock()->now();
    input_point_stamped.point = input_point;

    geometry_msgs::msg::PointStamped output_point_stamped;

    // Define timeout as tf2::Duration
    tf2::Duration timeout = tf2::durationFromSec(0.1);

    try
        {
        output_point_stamped = tf_buffer_.transform(
        input_point_stamped, 
        to_frame, 
        tf2::TimePointZero,   // Use TimePointZero for the latest available transform
        from_frame, 
        timeout
        );
        }
    catch (tf2::TransformException &ex)
        {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", from_frame.c_str(), to_frame.c_str(), ex.what());
        }

    return output_point_stamped.point;
}

*/

void ObjDetect::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    auto segments = countSegments(scan);
}

void ObjDetect::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    currentOdom = *odom;
}

std::vector<std::vector<geometry_msgs::msg::Point>> ObjDetect::countSegments(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    geometry_msgs::msg::Point current, previous;
    std::vector<std::vector<geometry_msgs::msg::Point>> segmentVector;
    std::vector<geometry_msgs::msg::Point> currentSegment;
    bool segStarted = false;

    for (size_t i = 1; i < scan->ranges.size(); i++)
    {
        if (std::isinf(scan->ranges.at(i - 1)) || std::isnan(scan->ranges.at(i - 1)))
            continue;

        while (i < scan->ranges.size() && (!std::isnan(scan->ranges.at(i))) && (scan->ranges.at(i) < scan->range_max))
        {
            float previousAngle = scan->angle_min + scan->angle_increment * (i - 1);
            previous.x = scan->ranges.at(i - 1) * cos(previousAngle);
            previous.y = scan->ranges.at(i - 1) * sin(previousAngle);

            float currentAngle = scan->angle_min + scan->angle_increment * i;
            current.x = scan->ranges.at(i) * cos(currentAngle);
            current.y = scan->ranges.at(i) * sin(currentAngle);

            float dist = hypot((current.x - previous.x), (current.y - previous.y));

            if (dist < 0.075)
            {
                if (!segStarted)
                {
                    segStarted = true;
                    currentSegment.clear();
                }
                currentSegment.push_back(localToGlobal(currentOdom, current));
            }
            else
            {

                break;
            }
            i++;
        }

        if (segStarted && !currentSegment.empty())
        {
            segmentVector.push_back(currentSegment);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
            if (!isThisAWall(currentSegment)) {
                if (!isThisACorner(currentSegment)) {
                    detectCylinder(currentSegment);
                } else if (isThis90(currentSegment)) {
                    //std::cout << "90-degree corner detected" << std::endl;
                    detectSquare(currentSegment);
                } else {
                    UnknownOBJ(currentSegment);
                }

            }
            segStarted = false;
        }

    }

    return segmentVector;
}

void ObjDetect::detectCylinder(const std::vector<geometry_msgs::msg::Point> &segment)
{
    if (segment.size() >= 6 && !isThisACorner(segment))
    {
        const auto &p1 = segment.front();
        const auto &p2 = segment.at(segment.size() / 2);
        const auto &p3 = segment.back();

        double a = hypot(p2.x - p1.x, p2.y - p1.y);
        double b = hypot(p3.x - p2.x, p3.y - p2.y);
        double c = hypot(p3.x - p1.x, p3.y - p1.y);

        double cosTheta = (a * a + b * b - c * c) / (2 * a * b);
        double theta = acos(cosTheta);
        double R = c / (2 * fabs(sin(theta / 2)));

        double targetRadius = 0.15;
        double tolerance_ = 0.02;

        if (fabs(R - targetRadius) < tolerance_)
        {
            geometry_msgs::msg::Point centre = findCentre(p1, p3, targetRadius);
            
            if (std::isnan(centre.x) || std::isnan(centre.y) || std::isnan(centre.z)) {
                // RCLCPP_WARN(this->get_logger(), "Skipping marker: detected NaN in center coordinates.");
                return;
            }

            // New: verify all points lie close to this radius
            bool allCloseToRadius = true;
            for (const auto &pt : segment)
            {
                double dist = hypot(pt.x - centre.x, pt.y - centre.y);
                if (fabs(dist - targetRadius) > tolerance_)
                {
                    allCloseToRadius = false;
                    break;
                }
            }

            if (allCloseToRadius && (!checkExisting(centre) || firstCent))
            {
                firstCent = false;
                centres.push_back(centre);
                RCLCPP_INFO(this->get_logger(), "Circle with radius ~%.2fm detected. Center: x = %.2f, y = %.2f, z = %.2f", targetRadius, centre.x, centre.y, centre.z);
                marker_pub_->publish(produceMarkerCylinder(centre, visualization_msgs::msg::Marker::CYLINDER, "black"));
            }
        }   
    }
}

void ObjDetect::detectSquare(const std::vector<geometry_msgs::msg::Point> &segment)
{
    // Get points from the segment
    const auto &p1 = segment.front();
    const auto &p2 = segment.at(segment.size() / 2);
    const auto &p3 = segment.back();

    // Compute side lengths
    double sideA = hypot(p2.x - p1.x, p2.y - p1.y);
    double sideB = hypot(p3.x - p2.x, p3.y - p2.y);

    double targetLength = 0.22;
    double tolerance = 0.05;

    if (isThis90(segment) &&
        fabs(sideA - targetLength) < tolerance &&
        fabs(sideB - targetLength) < tolerance)
    {
        geometry_msgs::msg::Point corner = p2;
        if (std::isnan(corner.x) || std::isnan(corner.y) || std::isnan(corner.z)) {
            // RCLCPP_WARN(this->get_logger(), "Skipping marker: detected NaN in square corner coordinates.");
            return;
        }
        
        // Optional: check if corner is already known to avoid duplicates
        if (!checkExisting(corner) || firstCent)
        {
            firstCent = false;
            centres.push_back(corner);

            RCLCPP_INFO(this->get_logger(), "Square corner detected at x = %.2f, y = %.2f", corner.x, corner.y);

            // Reuse the cylinder marker for now with a different color or style
            auto marker = produceMarkerCylinder(corner, visualization_msgs::msg::Marker::CUBE, "red");
            marker_pub_->publish(marker);
        }
    }
}

void ObjDetect::UnknownOBJ(const std::vector<geometry_msgs::msg::Point> &segment)
{
    // Check if segment is too close (any point closer than 0.2 m)
    bool tooClose = false;
    for (const auto &pt : segment)
    {
        double dist = hypot(pt.x, pt.y);
        if (dist < 0.2)
        {
            tooClose = true;
            break;
        }
    }
    if (!tooClose)
    return;

    // Calculate segment centroid
    geometry_msgs::msg::Point centroid;
    centroid.x = 0.0;
    centroid.y = 0.0;
    centroid.z = 0.0;

    for (const auto &pt : segment)
    {
        centroid.x += pt.x;
        centroid.y += pt.y;
        centroid.z += pt.z;
    }

    centroid.x /= segment.size();
    centroid.y /= segment.size();
    centroid.z /= segment.size();

    if (!checkExisting(centroid) || firstCent)
    {
        firstCent = false;
        centres.push_back(centroid);

        RCLCPP_INFO(this->get_logger(), "unknown object detected at x = %.2f, y = %.2f", centroid.x, centroid.y);

        // Reuse the cylinder marker for now with a different color or style
        auto marker = produceMarkerCylinder(centroid, visualization_msgs::msg::Marker::CYLINDER, "green");
        marker_pub_->publish(marker);
    }

}

geometry_msgs::msg::Point ObjDetect::findCentre(geometry_msgs::msg::Point P1, geometry_msgs::msg::Point P2, double r)
{
    geometry_msgs::msg::Point centre;
    double mx = (P1.x + P2.x) / 2.0;
    double my = (P1.y + P2.y) / 2.0;
    double d = hypot(P2.x - P1.x, P2.y - P1.y);
    double h = sqrt(r * r - (d / 2.0) * (d / 2.0));
    double dx = -(P2.y - P1.y) / d;
    double dy = (P2.x - P1.x) / d;

    double c1x = mx + h * dx;
    double c1y = my + h * dy;
    double c2x = mx - h * dx;
    double c2y = my - h * dy;

    double d2 = hypot(currentOdom.pose.pose.position.x - c1x, currentOdom.pose.pose.position.y - c1y);
    double d3 = hypot(currentOdom.pose.pose.position.x - c2x, currentOdom.pose.pose.position.y - c2y);

    if (d2 > d3)
        {
            centre.x = c1x;
            centre.y = c1y;
            centre.z = currentOdom.pose.pose.position.z;
        }
    else
        {
            centre.x = c2x;
            centre.y = c2y;
            centre.z = currentOdom.pose.pose.position.z;
        }
    return centre;
}

bool ObjDetect::checkExisting(geometry_msgs::msg::Point centre)
{
    duplicate_threshold_ = 0.3;
    for (const auto &existing_centre : centres)
    {
        if (hypot(centre.x - existing_centre.x, centre.y - existing_centre.y) < duplicate_threshold_)
            return true;
    }
    return false;
}

void ObjDetect::visualizeSegment(const std::vector<geometry_msgs::msg::Point> &segment)
{
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->get_clock()->now();
    line_marker.ns = "cylinder_markers";
    line_marker.id = ct_++;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.05;
    line_marker.color.a = 0.8;
    line_marker.color.g = 1.0;

    for (const auto &point : segment)
        line_marker.points.push_back(point);

    marker_pub_->publish(line_marker);
}

visualization_msgs::msg::Marker ObjDetect::produceMarkerCylinder(geometry_msgs::msg::Point pt, int type, const std::string& colour)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "cylinder_markers";
    marker.id = ct_++;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = pt;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    if (colour == "black") {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    } else if (colour == "red") {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    } else if (colour == "green") {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    } else {
        // Default to black if unrecognized
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    return marker;
}


geometry_msgs::msg::Point ObjDetect::localToGlobal(const nav_msgs::msg::Odometry &global, const geometry_msgs::msg::Point &local)
{
    geometry_msgs::msg::Point pt;
    double yaw = tf2::getYaw(global.pose.pose.orientation);
    pt.x = global.pose.pose.position.x + (local.x * cos(yaw) - local.y * sin(yaw));
    pt.y = global.pose.pose.position.y + (local.x * sin(yaw) + local.y * cos(yaw));
    pt.z = 0;
    return pt;
}

//Testing w/ tf2: updated L2G
/*
geometry_msgs::msg::Point ObjDetect::localToGlobal(const nav_msgs::msg::Odometry &global, 
    const geometry_msgs::msg::Point &local)
{
return transformPoint(local, "odom", "map");
}
*/
bool ObjDetect::isThisAWall(const std::vector<geometry_msgs::msg::Point> &segment)
{
    return hypot(segment.front().x - segment.back().x, segment.front().y - segment.back().y) > 0.6;
}

bool ObjDetect::isThisACorner(const std::vector<geometry_msgs::msg::Point> &segment)
{
    float a = hypot(segment.back().x - segment[segment.size() / 2].x, segment.back().y - segment[segment.size() / 2].y);
    float b = hypot(segment.front().x - segment[segment.size() / 2].x, segment.front().y - segment[segment.size() / 2].y);
    float c = hypot(segment.back().x - segment.front().x, segment.back().y - segment.front().y);
    float cosTheta = (a * a + b * b - c * c) / (2 * a * b);
    float theta = acos(cosTheta) * 180 / M_PI;

    return theta < 120;
}

bool ObjDetect::isThis90(const std::vector<geometry_msgs::msg::Point> &segment)
{
    if (segment.size() < 12){
        //std::cout << "Fail" << std::endl;
        return false;
    }

    float a = hypot(segment.back().x - segment[segment.size() / 2].x, segment.back().y - segment[segment.size() / 2].y);
    float b = hypot(segment.front().x - segment[segment.size() / 2].x, segment.front().y - segment[segment.size() / 2].y);
    float c = hypot(segment.back().x - segment.front().x, segment.back().y - segment.front().y);
    float cosTheta = (a * a + b * b - c * c) / (2 * a * b);
    float theta = acos(cosTheta) * 180 / M_PI;

    return (theta > 85.0 && theta < 95.0);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjDetect>());
    rclcpp::shutdown();
    return 0;
}